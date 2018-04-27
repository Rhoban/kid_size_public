#include "RobotByII.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rhoban_utils/timing/benchmark.h"
#include "Utils/OpencvUtils.h"
#include "Utils/ROITools.hpp"
#include "Utils/RotatedRectUtils.hpp"
#include "CameraState/CameraState.hpp"

#include <robocup_referee/constants.h>

#include <set>

/* TODO
- Why is the whole screen black sometimes?
- Add the filtering from SimpleRobotDetector here to improve the distance
precision
- Add a magenta vs cyan [-1, 1] score
 */

using rhoban_utils::Benchmark;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

namespace Vision {
namespace Filters {

struct greater {
  template <class T>
  bool operator()(T const &a, T const &b) const {
    return a > b;
  }
};

RobotByII::RobotByII() : Filter("RobotByII") {}

std::string RobotByII::getClassName() const { return "RobotByII"; }

int RobotByII::expectedDependencies() const { return 7; }

void RobotByII::setParameters() {
  actualRobotHeight = ParamFloat(0.6, 0.01, 2);
  actualRobotWidth = ParamFloat(0.3, 0.01, 1);
  widthScale = ParamFloat(2.0, 0.1, 10.0);
  aboveRatio = ParamFloat(2.0, 0.1, 10.0);
  belowRatio = ParamFloat(2.0, 0.1, 10.0);
  roiRatio = ParamFloat(2.0, 1.0, 10.0);
  belowCoeff = ParamFloat(1.0, 0.0, 10.0);
  sideCoeff = ParamFloat(1.0, 0.0, 10.0);
  robotWidth = ParamFloat(0.3, 0.01, 1);
  boundaryWidthRatio = ParamFloat(3.0, 1.0, 5.0);
  greenThreshold = ParamFloat(100, 0, 255);
  whiteThreshold = ParamFloat(100, 0, 255);
  minWidth = ParamFloat(2.0, 0.5, 20.0);
  minScore = ParamFloat(255.0, 0.0, 510.);
  darkMatters = ParamFloat(2.0, 0.0, 10.0);
  tooCloseToBeTrue = ParamFloat(0.4, 0.0, 2.0);
  maxRois = ParamInt(4, 1, 100);
  decimationRate = ParamInt(1, 1, 20);
  tagLevel = ParamInt(0, 0, 2);

  params()->define<ParamFloat>("widthScale", &widthScale);
  params()->define<ParamFloat>("aboveRatio", &aboveRatio);
  params()->define<ParamFloat>("belowRatio", &belowRatio);
  params()->define<ParamFloat>("roiRatio", &roiRatio);
  params()->define<ParamFloat>("belowCoeff", &belowCoeff);
  params()->define<ParamFloat>("sideCoeff", &sideCoeff);
  params()->define<ParamFloat>("boundaryWidthRatio", &boundaryWidthRatio);
  params()->define<ParamFloat>("minWidth", &minWidth);
  params()->define<ParamFloat>("minScore", &minScore);
  params()->define<ParamFloat>("darkMatters", &darkMatters);
  params()->define<ParamFloat>("greenThreshold", &greenThreshold);
  params()->define<ParamFloat>("whiteThreshold", &whiteThreshold);
  params()->define<ParamFloat>("tooCloseToBeTrue", &tooCloseToBeTrue);
  params()->define<ParamFloat>("robotWidth", &robotWidth);
  params()->define<ParamFloat>("actualRobotHeight", &actualRobotHeight);
  params()->define<ParamFloat>("actualRobotWidth", &actualRobotWidth);
  params()->define<ParamInt>("maxRois", &maxRois);
  params()->define<ParamInt>("decimationRate", &decimationRate);
  params()->define<ParamInt>("tagLevel", &tagLevel);
}

// Note : this code is pretty much the same thing than GoalByII with a few changes.
// Here though, we do not need the outputed ROIs, I kept them in case we decide to try
// the neural network on them too.

void RobotByII::process() {
  // Get names of dependencies
  const std::string &whiteName = _dependencies[0];
  const std::string &greenName = _dependencies[1];
  const std::string &darkName = _dependencies[2];
  const std::string &widthProviderName = _dependencies[3];
  const std::string &clippingName = _dependencies[4];
  const std::string &greenDensityName = _dependencies[5];
  const std::string &whiteDensityName = _dependencies[6];
  
  // Import source matrix and update size
  const cv::Mat &whiteII = *(getDependency(whiteName).getImg());
  const cv::Mat &greenII = *(getDependency(greenName).getImg());
  const cv::Mat &darkII = *(getDependency(darkName).getImg());
  const cv::Mat &widthImg = *(getDependency(widthProviderName).getImg());
  const cv::Mat &clipping = *(getDependency(clippingName).getImg());
  cv::Mat greenDensity = *(getDependency(greenDensityName).getImg());
  cv::Mat whiteDensity = *(getDependency(whiteDensityName).getImg());
  // Integral images have 1 line and 1 column more than resulting image
  rows = whiteII.rows - 1;
  cols = whiteII.cols - 1;

  clearRois();

  // Checks on size consistency
  if (whiteII.cols != greenII.cols || whiteII.cols != darkII.cols || whiteII.cols != (widthImg.cols + 1) ||
      clipping.cols < 1) {
    throw std::runtime_error("Invalid input sizes in RobotByII.");
  }
  if (greenDensity.cols != whiteDensity.cols) {
    throw std::runtime_error("Error in RobotByII, greenDensity and whiteDensity must have the sime size");
  }

  float clippingRatioCols = cols / float(clipping.cols);
  float clippingRatioRows = rows / float(clipping.rows);

  // boundaries are used to analyze if areas are colliding
  std::vector<double> scores;
  // boundaries are used to analyze if areas are colliding
  std::vector<cv::Rect> boundaryPatches;

  std::vector<cv::Point2f> centers;

  double imgMinScore = 0;
  double imgMaxScore = 0;

  const cv::Size &srcSize = whiteII.size();

  // Computing score matrix and ROI at once
  cv::Mat scoresImg;
  if (tagLevel > 0) {
    scoresImg = cv::Mat(rows, cols, CV_32SC1, cv::Scalar(0.0));
  }

  /*
  //TEMP
  img() = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
  decimationRate = 1;
  for (int y = 0; y * decimationRate < rows; y ++) {
    for (int x = 0; x * decimationRate < cols; x ++) {
      // Computing limits of the current area
      int start_x = x * decimationRate;
      int start_y = y * decimationRate;
      int end_x = (x+1) * decimationRate;
      int end_y = (y+1) * decimationRate;
      if (end_x >= cols) end_x = cols-1;
      if (end_y >= rows) end_y = rows-1;

      // Getting the middle point
      int center_x = (start_x + end_x) / 2;
      int center_y = (start_y + end_y) / 2;

      // Computing patches
      // Gettting the size of the ball at the current point and deducing the expected
      // robot width
      float ballRadius = widthImg.at<float>(center_y,center_x);
      float factor = 1;
      uchar colorValue = 0;
      if (factor*ballRadius > 255) {
        colorValue = 255;
      } else if (ballRadius < 0) {
        throw std::runtime_error(std::string("Negative radius ") +  std::to_string(ballRadius));
      } else {
        colorValue = factor*ballRadius;
      }
      img().at<cv::Vec3b>(center_y, center_x)[2] = colorValue;
    }
  }
  return;
  //END OF TEMP
  */

  for (int y = 0; y * decimationRate < rows; y++) {
    for (int x = 0; x * decimationRate < cols; x++) {
      if (clipping.at<uchar>(y * decimationRate / clippingRatioRows, x * decimationRate / clippingRatioCols) == 0) {
        // Clipped out
        continue;
      }

      // Computing limits of the current area
      int start_x = x * decimationRate;
      int start_y = y * decimationRate;
      int end_x = (x + 1) * decimationRate;
      int end_y = (y + 1) * decimationRate;
      if (end_x >= cols) end_x = cols - 1;
      if (end_y >= rows) end_y = rows - 1;

      // Getting the middle point
      int center_x = (start_x + end_x) / 2;
      int center_y = (start_y + end_y) / 2;

      // Computing patches
      // Gettting the size of the ball at the current point and deducing the expected
      // robot width
      float ballRadius = widthImg.at<float>(center_y, center_x);
      float ballRadiusM = Constants::field.ballRadius;
      float m2pixels = ballRadius / ballRadiusM;
      float width = robotWidth * m2pixels;
      cv::Rect boundary_patch = getBoundaryPatch(center_x, center_y, width);
      // Skip ROI if it doesn't fit entirely in the image and use a '0' score
      if (!Utils::isContained(boundary_patch, srcSize) ||
          width * widthScale <= minWidth) {
        if (tagLevel > 0) {
          fillScore(scoresImg, -20, start_x, end_x, start_y, end_y);
        }
        continue;
      }
      cv::Rect above_patch = getAbovePatch(center_x, center_y, width);
      cv::Rect below_patch = getBelowPatch(center_x, center_y, width);
      cv::Rect above_right_patch = getAboveRightPatch(center_x, center_y, width);
      cv::Rect above_left_patch = getAboveLeftPatch(center_x, center_y, width);
      
      // Skip empty patches
      if (above_patch.area() == 0 || below_patch.area() == 0 || above_right_patch.area() == 0 ||
          above_left_patch.area() == 0) {
        fillScore(scoresImg, -20, start_x, end_x, start_y, end_y);
        continue;
      }

      // score = (Above - Below) + (Above - Above_right) + (Above - Above_left)
      double above_score = getPatchScore(above_patch, whiteII, greenII, darkII);
      double below_score = getPatchScore(below_patch, whiteII, greenII, darkII);
      double R = getPatchScore(above_right_patch, whiteII, greenII, darkII);
      double L = getPatchScore(above_left_patch, whiteII, greenII, darkII);
      double totalCoeff = belowCoeff + sideCoeff;
      // Normalizing back to [-255, 255]
      double score =
          (belowCoeff * (above_score - below_score) + sideCoeff * (2 * above_score - L - R)) / (3 * totalCoeff);

      // Write score in scores map
      if (tagLevel > 0) {
        fillScore(scoresImg, (int)score, start_x, end_x, start_y, end_y);
        if (tagLevel > 1) {
          // Update score boundaries
          imgMaxScore = 255;
          imgMinScore = -255;
        } else {
          // Update score boundaries
          if (score > imgMaxScore) {
            imgMaxScore = score;
          }
          if (score < imgMinScore) {
            imgMinScore = score;
          }
        }
      }

      // If score of patch is too low to use it, skip
      if (score < minScore) {
        continue;
      }
      // Establishing:
      // - List of dominated ROIs
      // - Worst score
      // If current candidate is dominated, then stop the process and ignore current candidate
      std::vector<size_t> dominated_rois;
      bool dominated = false;
      double worstScore = score;
      int worstId = -1;
      for (size_t id = 0; id < boundaryPatches.size(); id++) {
        // If there is an overlap with 'id'
        if (Utils::isOverlapping(boundaryPatches[id], boundary_patch)) {
          // If new region is better, then it dominates 'id'
          if (scores[id] < score) {
            dominated_rois.push_back(id);
          }
          // If new region is not better, simply stop the process and ignore the region
          else {
            dominated = true;
            break;
          }
        }
        // If score is lower than previously met, update
        if (worstScore > scores[id]) {
          worstScore = scores[id];
          worstId = id;
        }
      }
      // If the new roi is dominated, ignore it
      if (dominated) {
      }
      // No areas directly dominated
      else if (dominated_rois.size() == 0) {
        // If there is enough space remaining, push element
        if ((int)scores.size() < maxRois) {
          scores.push_back(score);
          boundaryPatches.push_back(boundary_patch);
          centers.push_back(cv::Point2f(center_x, center_y));
        }
        // If there is not enough space: replace worst ROI
        else if (worstId != -1) {
          scores[worstId] = score;
          boundaryPatches[worstId] = boundary_patch;
          centers.push_back(cv::Point2f(center_x, center_y));
        }
      }
      // If one area is dominated, replace it
      else if (dominated_rois.size() == 1) {
        size_t dominated_idx = dominated_rois[0];
        scores[dominated_idx] = score;
        boundaryPatches[dominated_idx] = boundary_patch;
        centers.push_back(cv::Point2f(center_x, center_y));
      }
      // If more than one area is dominated, remove all dominated areas and add current one
      else {
        std::vector<bool> removeFlags(scores.size(), false);
        for (size_t roi_id : dominated_rois) {
          removeFlags[roi_id] = true;
        }
        std::vector<double> oldScores = scores;
        std::vector<cv::Rect> oldBoundaries = boundaryPatches;
        scores.clear();
        boundaryPatches.clear();
        for (size_t roi_idx = 0; roi_idx < oldScores.size(); roi_idx++) {
          if (!removeFlags[roi_idx]) {
            scores.push_back(oldScores[roi_idx]);
            boundaryPatches.push_back(oldBoundaries[roi_idx]);
          }
        }
        scores.push_back(score);
        boundaryPatches.push_back(boundary_patch);
        centers.push_back(cv::Point2f(center_x, center_y));
      }
    }
  }

  // Updating the robots list
  robots.clear();
  for (size_t roi_idx = 0; roi_idx < scores.size(); roi_idx++) {
    //addRoi(scores[roi_idx], Utils::toRotatedRect(roiPatches[roi_idx]));
    // The center of the roi is the point with the highest local score
    cv::Point2f p = centers[roi_idx];
    // Normalizing the point in [0, 1]
    p.x = p.x / cols;
    p.y = p.y / rows;
    robots.push_back(std::pair<cv::Point2f, float>(p, scores[roi_idx]));
  }
  
  if (tagLevel > 0) {
    img() = getHeatMap(scoresImg, imgMinScore, imgMaxScore);
    drawChosenOnes(img(), robots, cols, rows, 1);
  } else {
    img() = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  }
    // Filtering candidates
  filterCandidates(robots, greenDensity, whiteDensity);

  if (tagLevel > 0) {
    //for (auto & r : robots) {
      //std::cout << "(after) r.first = " << r.first << std::endl;
    //}

    drawChosenOnes(img(), robots, cols, rows, 2);
  }
}

cv::Mat RobotByII::getHeatMap(const cv::Mat &scores, double imgMinScore, double imgMaxScore) const {
  cv::Mat result(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  double diffScore = imgMaxScore - imgMinScore;
  if (diffScore > 0) {
    double factorBelow = 0;
    double factorAbove = 0;
    // Normalizing the scores between [0-255]
    if (imgMaxScore > 0) factorAbove = 255.0 / imgMaxScore;
    if (imgMinScore < 0) factorBelow = 255.0 / imgMinScore;
    // Going back to color
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
        int score = scores.at<int>(y, x);
        if (score > 0) {
          int intensity = (int)(score * factorAbove);
          result.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, intensity);
        } else {
          int intensity = (int)(score * factorBelow);
          result.at<cv::Vec3b>(y, x) = cv::Vec3b(intensity, 0, 0);
        }
      }
    }
  }
  return result;
}

cv::Rect_<float> RobotByII::getAbovePatch(int x, int y, float width) {
  float half_width = width * widthScale / 2.0;
  float height = width * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center - cv::Point2f(half_width, height), center + cv::Point2f(half_width, 0));
}

cv::Rect_<float> RobotByII::getAboveRightPatch(int x, int y, float width) {
  float low_limit = width * widthScale / 2.0;
  float far_limit = low_limit * boundaryWidthRatio;
  float height = 2 * low_limit * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center + cv::Point2f(low_limit, -height), center + cv::Point2f(far_limit, 0));
}

cv::Rect_<float> RobotByII::getAboveLeftPatch(int x, int y, float width) {
  float low_limit = width * widthScale / 2.0;
  float far_limit = low_limit * boundaryWidthRatio;
  float height = 2 * low_limit * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center - cv::Point2f(far_limit, height), center - cv::Point2f(low_limit, 0));
}

cv::Rect_<float> RobotByII::getBelowPatch(int x, int y, float width) {
  float half_width = width * widthScale / 2.0;
  float far_limit = half_width * boundaryWidthRatio;
  float below = width * widthScale * belowRatio;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center - cv::Point2f(far_limit, 0), center + cv::Point2f(far_limit, below));
}

cv::Rect_<float> RobotByII::getBoundaryPatch(int x, int y, float width) {
  float half_width = width * widthScale * boundaryWidthRatio / 2.0;
  float above = width * widthScale * aboveRatio;
  float below = width * widthScale * belowRatio;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center - cv::Point2f(half_width, above), center + cv::Point2f(half_width, below));
}

cv::Rect_<float> RobotByII::getROIPatch(int x, int y, float width) {
  float half_size = width * widthScale * roiRatio / 2.0;
  // Creating inner patch
  cv::Point2f center(x, y);
  return cv::Rect_<float>(center - cv::Point2f(half_size, half_size), center + cv::Point2f(half_size, half_size));
}

double RobotByII::getPatchScore(const cv::Rect &patch, const cv::Mat &whiteII, const cv::Mat &greenII,
                                 const cv::Mat &darkII) {
  // Use cropped rectangle
  cv::Rect cropped = Utils::cropRect(patch, cv::Size(cols, rows));

  // Return 0 score for empty areas
  if (cropped.area() == 0) return 0;

  // Top left and bottom right corners
  cv::Point2i tl, br;
  tl = cropped.tl();
  br = cropped.br();  // Offset has to be counted on image score

  int A, B, C, D;
  A = darkII.at<int>(tl.y, tl.x) - (whiteII.at<int>(tl.y, tl.x) + greenII.at<int>(tl.y, tl.x)) / 2.0;
  B = darkII.at<int>(tl.y, br.x) - (whiteII.at<int>(tl.y, br.x) + greenII.at<int>(tl.y, br.x)) / 2.0;
  C = darkII.at<int>(br.y, tl.x) - (whiteII.at<int>(br.y, tl.x) + greenII.at<int>(br.y, tl.x)) / 2.0;
  D = darkII.at<int>(br.y, br.x) - (whiteII.at<int>(br.y, br.x) + greenII.at<int>(br.y, br.x)) / 2.0;

  double area = cropped.area();
  return (A + D - B - C) / area;
}

void RobotByII::fillScore(cv::Mat &img, int score, int start_x, int end_x, int start_y, int end_y) {
  for (int y = start_y; y < end_y; y++) {
    for (int x = start_x; x < end_x; x++) {
      img.at<int>(y, x) = score;
    }
  }
}

// Using a copy of scores on purpose
  void RobotByII::drawChosenOnes(cv::Mat &img, std::vector<std::pair<cv::Point2f, float> > robots, int cols, int rows, int mode) {
  // std::sort(scores.begin(), scores.end(), greater());
  for (unsigned int i = 0; i < robots.size(); i++) {
    cv::Point2f temp(robots[i].first.x * cols, robots[i].first.y * rows);
    cv::Scalar color;
    if (mode == 1) {
      // Yellow for raw points
      color = cv::Scalar(0, robots[i].second, robots[i].second);
    } else if (mode == 2) {
      // Green for final points
      color = cv::Scalar(0, robots[i].second, 0);
    } else {
      color = cv::Scalar(0, robots[i].second, 0);
    }
    cv::circle(img, temp, 5, color, -1);
    // img.at<cv::Vec3b>(robots[i].first.y, robots[i].first.x) = cv::Vec3b(0, robots[i].second, 0);
    //std::cout << "Score = " << robots[i].second << std::endl;
  }
}
  
  void RobotByII::filterCandidates(std::vector<std::pair<cv::Point2f, float> > & robots, cv::Mat & greenDensity, cv::Mat & whiteDensity) {
    // For each point, find the corresponding bottom point (that enters a green or a white area)
    std::vector<int> deleteMe;
    int index = -1;
    for (auto & r : robots) {
      index++;
      cv::Point2f p = r.first;
      //std::cout << "p = " << p << std::endl;
      cv::Point2i bottomLimit;
      if (isItMyself(p)) {
        // The point is considered too close to be a robot (it might be our own feet or arm)
        //std::cout << "Too close !"<< std::endl;
        deleteMe.push_back(index);
        continue;
      }
      float score =
          findLimit(&bottomLimit, greenDensity, whiteDensity, greenThreshold, whiteThreshold,
                                         p.x*greenDensity.cols, p.y*greenDensity.rows, 0, 1);

      if (score < 0.1) {
        deleteMe.push_back(index);
      } else {
        r.first.x = bottomLimit.x/(float)greenDensity.cols;
        r.first.y = bottomLimit.y/(float)greenDensity.rows;
      }

    }
    for (int i = deleteMe.size() - 1; i > -1; i--) {
      // Erasing from the end of the vector to the start, so the smaller indexes
      // don't change.
      robots.erase(robots.begin() + deleteMe[i]);
    }
    robots = shadowRobots(robots);
  }

  float RobotByII::findLimit(cv::Point2i * result, cv::Mat &green, cv::Mat &white, int threshGreen, int threshWhite, int x, int y,
                                     int xDir, int yDir) {
  // Default quality when we hit a side of the screen
  float quality = 0.5;
  *result = cv::Point2i(0, 0);
  if (white.at<uchar>(y, x) >= threshWhite) {
    // Testing if we're already a white heavy place (typically the ball)
    quality = 0.0;
    return quality;
  }
  while (true) {
    x = x + xDir;
    y = y + yDir;
    // Exit conditions
    bool gonaBreak = false;
    if (x < 0) {
      x = 0;
      gonaBreak = true;
    } else if (x > (green.cols - 1)) {
      x = green.cols - 1;
      gonaBreak = true;
    }
    if (y < 0) {
      y = 0;
      gonaBreak = true;
    } else if (y > (green.rows - 1)) {
      y = green.rows - 1;
      gonaBreak = true;
    }
    if (gonaBreak) {
      break;
    }
    if (green.at<uchar>(y, x) >= threshGreen || white.at<uchar>(y, x) >= threshWhite) {
      quality = 1.0;
      break;
    }
  }
  *result = cv::Point2i(x, y);
  return quality;
}

  /*
   * The idea here is to keep the closest robot (point) and to merge all the robots that fit into our
   * model of a robot (a rectangle) into that first robot. The robots that did not merge will create 
   * another cluster. We do this because the simple detection method gives plenty of results for the 
   * same robot (the legs and arms are detected as robots for example)
   */
  std::vector<std::pair<cv::Point2f, float> > RobotByII::shadowRobots(std::vector<std::pair<cv::Point2f, float> > & robots) {
  std::vector<std::pair<cv::Point2f, float> > finalRobots;
  if (robots.size() < 1) {
    return finalRobots;
  }
  
  sortRobots(robots);
  while (robots.size() > 0) {
    finalRobots.push_back(robots[0]);
    cv::Point2f choosen = robots[0].first;
    robots.erase(robots.begin());
    // The robots are sorted, the first one is considered to be the closest. If becomes a final
    // robot and destroys near by robots
    destroyNear(choosen, robots);
  }
  return finalRobots;
}

  int RobotByII::destroyNear(cv::Point2f center, std::vector<std::pair<cv::Point2f, float> > & robots) {
    std::vector<int> deleteMe;

    for (unsigned int i = 0; i < robots.size(); i++) {
      if (isNear(center, robots[i].first)) {
        deleteMe.push_back(i);
      }
    }
    
    for (int i = deleteMe.size() - 1; i > -1; i--) {
      // Erasing from the end of the vector to the start, so the smaller indexes
      // don't change.
      robots.erase(robots.begin() + deleteMe[i]);
    }
    return deleteMe.size();
  }
  
  bool RobotByII::isNear(cv::Point2f center, cv::Point2f p) {
    // Points are in % of the image
    int radiusMin = 0;
    int radiusMax = 0;
    int fakeSizeWidth = 640;
    int fakeSizeHeight = 480;
    // PLEASE
    cv::Point2f temp(center.x*fakeSizeWidth, center.y*fakeSizeHeight);
    getCS().ballInfoFromPixel(temp, fakeSizeWidth, fakeSizeHeight, &radiusMin, &radiusMax, 1.0);
    // Using the ball size to convert from pixels to m
    float M2PIXEL = ((radiusMax + radiusMin)/2)/(Constants::field.ballRadius*fakeSizeHeight);
    
    if ((center.x - M2PIXEL*actualRobotWidth/2) > p.x || (center.x + M2PIXEL*actualRobotWidth/2) < p.x) {
      return false;
    }
    if ((center.y - M2PIXEL*actualRobotHeight) > p.y) {
      return false;
    }

    return true; 
  }

  void RobotByII::sortRobots(std::vector<std::pair<cv::Point2f, float> > & robots) {
    sort(robots.begin(), robots.end(),
         [](const std::pair<cv::Point2f, float> &lhs, const std::pair<cv::Point2f, float> &rhs) {
           return lhs.first.y > rhs.first.y;
         });
  }

  bool RobotByII::isItMyself(cv::Point2f p) {
    // Going from pixels to world (in robot frame) (m)
    try {
      cv::Point2f pos = getCS().robotPosFromImg(p.x, p.y, 1, 1, true)*100;
      //std::cout << "In image " << p << ", in robot frame " << pos << std::endl;
      float dist = sqrt(pos.x*pos.x + pos.y*pos.y);
      if (pos.x > 1) {
        float angle = rad2deg(abs(atan2(pos.y, pos.x)));
        if (angle > 40) {
          return true;
        }
      } else {
        return true;
      }

      return dist < tooCloseToBeTrue;
    } catch (const std::runtime_error & exc) {
      // When robotPosFromImg asks for a point above horizon, it is sure that it
      // is not myself
      return false;
    }
  }
  
}
}
