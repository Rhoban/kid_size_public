#include "ObstacleByII.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rhoban_utils/timing/benchmark.h"
#include "Utils/OpencvUtils.h"
#include "Utils/ROITools.hpp"
#include "Utils/RotatedRectUtils.hpp"
#include "CameraState/CameraState.hpp"

#include <rhoban_utils/logging/logger.h>

#include <set>

using rhoban_utils::Benchmark;

static rhoban_utils::Logger logger("ObstacleByII");

namespace Vision {
namespace Filters {

ObstacleByII::ObstacleByII() : Filter("ObstacleByII")
{
}


std::string ObstacleByII::getClassName() const {
  return "ObstacleByII";
}

int ObstacleByII::expectedDependencies() const {
  return 2;
}

void ObstacleByII::setParameters() {
  widthScale = ParamFloat(2.0,0.1,10.0);
  aboveRatio = ParamFloat(2.0,0.1,10.0);
  belowRatio = ParamFloat(2.0,0.1,10.0);
  roiRatio   = ParamFloat(2.0,1.0,10.0);
  belowCoeff = ParamFloat(1.0,0.0,10.0);
  sideCoeff  = ParamFloat(1.0,0.0,10.0);
  boundaryWidthRatio = ParamFloat(3.0,1.0,5.0);
  boundaryHeightRatio = ParamFloat(0.8,0.2,2.0);
  minWidth = ParamFloat(2.0,0.5,100.0);
  minScore = ParamFloat(127.0,0.0,255.);
  maxRois = ParamInt(6,1,100);
  decimationRate = ParamInt(4,1,20);
  tagLevel = ParamInt(0,0,1);

  params()->define<ParamFloat>("widthScale", &widthScale);
  params()->define<ParamFloat>("aboveRatio", &aboveRatio);
  params()->define<ParamFloat>("belowRatio", &belowRatio);
  params()->define<ParamFloat>("roiRatio", &roiRatio);
  params()->define<ParamFloat>("belowCoeff", &belowCoeff);
  params()->define<ParamFloat>("sideCoeff" , &sideCoeff );
  params()->define<ParamFloat>("boundaryWidthRatio", &boundaryWidthRatio);
  params()->define<ParamFloat>("boundaryHeightRatio", &boundaryHeightRatio);
  params()->define<ParamFloat>("minWidth", &minWidth);
  params()->define<ParamFloat>("minScore", &minScore);
  params()->define<ParamInt>("maxRois", &maxRois);
  params()->define<ParamInt>("decimationRate", &decimationRate);
  params()->define<ParamInt>("tagLevel", &tagLevel);
}

void ObstacleByII::process() {
  // Get names of dependencies
  const std::string & greenName = _dependencies[0];
  const std::string & widthProviderName = _dependencies[1];
  // Import source matrix and update size
  const cv::Mat & greenII  = *(getDependency(greenName).getImg());
  const cv::Mat & widthImg = *(getDependency(widthProviderName).getImg());

  // Integral images have 1 line and 1 column more than resulting image
  rows = greenII.rows - 1;
  cols = greenII.cols - 1;

  // TODO: add checks on size consistency

  clearRois();

  // boundaries are used to analyze if areas are colliding
  std::vector<double> scores;
  // boundaries are used to analyze if areas are colliding
  std::vector<cv::Rect> boundaryPatches;
  // roiPatches are used to create the region of interest:
  // - They tend to be larger because recognizing robots require context
  std::vector<cv::Rect> roiPatches;

  double imgMinScore = 0;
  double imgMaxScore = 0;

  const cv::Size & srcSize = greenII.size();

  Benchmark::open("computing decimated scores");
  // Computing score matrix and ROI at once
  cv::Mat scoresImg;
  if (tagLevel > 0) {
    scoresImg = cv::Mat(rows, cols, CV_32SC1, cv::Scalar(0.0));
  }
  for (int y = 0; y * decimationRate < rows; y ++) {
    for (int x = 0; x * decimationRate < cols; x ++) {
      // Computing limits of the current area
      int start_x = x * decimationRate;
      int start_y = y * decimationRate;
      int end_x = (x+1) * decimationRate;
      int end_y = (y+1) * decimationRate;
      if (end_x >= cols) end_x = cols-1;
      if (end_y >= rows) end_y = rows-1;

      // Getting the middle point and the width of the image
      int center_x = (start_x + end_x) / 2;
      int center_y = (start_y + end_y) / 2;
      float width = widthImg.at<float>(center_y,center_x);

      bool tooThin = width * widthScale < minWidth;

      if (tooThin) {
        if (tagLevel > 0) {
          fillScore(scoresImg, 0, start_x, end_x, start_y, end_y);
        }
        continue;
      }

      // Computing patches
      cv::Rect above_patch = getAbovePatch(center_x, center_y, width);
      cv::Rect below_patch = getBelowPatch(center_x, center_y, width);
      cv::Rect above_right_patch = getAboveRightPatch(center_x, center_y, width);
      cv::Rect above_left_patch = getAboveLeftPatch(center_x, center_y, width);
      cv::Rect boundary_patch = getBoundaryPatch(center_x, center_y, width);
      cv::Rect roi_patch = getROIPatch(center_x, center_y, width);

      // If mode discard partial ROIs and boundaryPatch is not inside ROI:
      // Skip ROI and use a '0' score
      if (!Utils::isContained(above_patch, srcSize)    ||
          !Utils::isContained(boundary_patch, srcSize) ||
          !Utils::isContained(roi_patch, srcSize)) {
        if (tagLevel > 0) {
          fillScore(scoresImg, 0, start_x, end_x, start_y, end_y);
        }
        continue;
      }

      // Skip empty patches
      if (above_patch.area() == 0 || 
          below_patch.area() == 0 || 
          above_right_patch.area() == 0 || 
          above_left_patch.area() == 0) {
        continue;
      }

      // score = belowCoeff * (Above - Below)
      //         + sideCoeff * ((Above - Above_right) + (Above - Above_left))
      double above_score = getPatchScore(above_patch, greenII);
      double below_score = getPatchScore(below_patch, greenII);
      double R = getPatchScore(above_right_patch, greenII);
      double L = getPatchScore(above_left_patch, greenII);
      double totalCoeff = belowCoeff + sideCoeff;
      double score = (belowCoeff * (below_score - above_score)
                      + sideCoeff * (L + R - 2 * above_score)) / (totalCoeff);

      // Write score in scores map
      if (tagLevel > 0) {
        fillScore(scoresImg, (int)score, start_x, end_x, start_y, end_y);
        // Update score boundaries
        if (score > imgMaxScore) {
          imgMaxScore = score;
        }
        if (score < imgMinScore) {
          imgMinScore = score;
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
      if (dominated) {}
      // No areas directly dominated
      else if (dominated_rois.size() == 0) {
        // If there is enough space remaining, push element
        if ((int)scores.size() < maxRois) {
          scores.push_back(score);
          boundaryPatches.push_back(boundary_patch);
          roiPatches.push_back(roi_patch);
        }
        // If there is not enough space: replace worst ROI
        else if (worstId != -1) {
          scores[worstId] = score;
          boundaryPatches[worstId] = boundary_patch;
          roiPatches[worstId] = roi_patch;
        }
      }
      // If one area is dominated, replace it
      else if (dominated_rois.size() == 1) {
        size_t dominated_idx = dominated_rois[0];
        scores[dominated_idx] = score;
        boundaryPatches[dominated_idx] = boundary_patch;
        roiPatches[dominated_idx] = roi_patch;
      }
      // If more than one area is dominated, remove all dominated areas and add current one
      else {
        std::vector<bool> removeFlags(scores.size(), false);
        for (size_t roi_id : dominated_rois) {
          removeFlags[roi_id] = true;
        }
        std::vector<double> oldScores = scores;
        std::vector<cv::Rect> oldBoundaries = boundaryPatches;
        std::vector<cv::Rect> oldRois = roiPatches;
        scores.clear();
        boundaryPatches.clear();
        roiPatches.clear();
        for (size_t roi_idx = 0; roi_idx < oldScores.size(); roi_idx++) {
          if (!removeFlags[roi_idx]) {
            scores.push_back(oldScores[roi_idx]);
            boundaryPatches.push_back(oldBoundaries[roi_idx]);
            roiPatches.push_back(oldRois[roi_idx]);
          }
        }
        scores.push_back(score);
        boundaryPatches.push_back(boundary_patch);
        roiPatches.push_back(roi_patch);
      }
    }
  }

  Benchmark::close("computing decimated scores");

  for (size_t roi_idx = 0; roi_idx < scores.size(); roi_idx++){
    addRoi(scores[roi_idx], Utils::toRotatedRect(roiPatches[roi_idx]));
  }

  Benchmark::open("getHeatMap");
  if (tagLevel > 0) {
    img() = getHeatMap(scoresImg, imgMinScore, imgMaxScore);
  }
  else {
    img() = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
  }
  Benchmark::close("getHeatMap");
}

cv::Mat ObstacleByII::getHeatMap(const cv::Mat & scores,
                              double imgMinScore,
                              double imgMaxScore) const
{
  cv::Mat result(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
  double diffScore = imgMaxScore - imgMinScore;
  if (diffScore > 0) {  
    double factorBelow = 0;
    double factorAbove = 0;
    //Normalizing the scores between [0-255]
    if (imgMaxScore > 0) factorAbove = 255.0/imgMaxScore;
    if (imgMinScore < 0) factorBelow = 255.0/imgMinScore;
    // Going back to color
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
        int score = scores.at<int>(y, x);
        if (score > 0) {
          int intensity = (int)(score * factorAbove);
          result.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, intensity);
        } else  {
          int intensity = (int)(score * factorBelow);
          result.at<cv::Vec3b>(y, x) = cv::Vec3b(intensity, 0, 0);
        }
      }
    }
  }
  return result;
}

cv::Rect_<float> ObstacleByII::getAbovePatch(int x, int y, float width)
{
  float half_width = width * widthScale / 2.0;
  float height     = half_width * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center - cv::Point2f(half_width, height),
                          center + cv::Point2f(half_width, 0));
}

cv::Rect_<float> ObstacleByII::getAboveRightPatch(int x, int y, float width)
{
  float low_limit = width * widthScale / 2.0;
  float far_limit = low_limit * boundaryWidthRatio;
  float height    = low_limit * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center + cv::Point2f(low_limit, -height),
                          center + cv::Point2f(far_limit, 0));
}

cv::Rect_<float> ObstacleByII::getAboveLeftPatch(int x, int y, float width)
{
  float low_limit = width * widthScale / 2.0;
  float far_limit = low_limit * boundaryWidthRatio;
  float height    = low_limit * aboveRatio;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center - cv::Point2f(far_limit, height),
                          center - cv::Point2f(low_limit, 0));
}

cv::Rect_<float> ObstacleByII::getBelowPatch(int x, int y, float width)
{
  float half_width = width * widthScale / 2.0;
  float far_limit = half_width * boundaryWidthRatio;
  float below = width * widthScale * belowRatio;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center - cv::Point2f(far_limit, 0),
                          center + cv::Point2f(far_limit, below));
}

cv::Rect_<float> ObstacleByII::getBoundaryPatch(int x, int y, float width)
{
  float half_width = width * widthScale * boundaryWidthRatio / 2.0;
  float half_height = width * widthScale * boundaryHeightRatio / 2.0;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center - cv::Point2f(half_width, half_height),
                          center + cv::Point2f(half_width, half_height));
}

cv::Rect_<float> ObstacleByII::getROIPatch(int x, int y, float width)
{
  float half_size = width * widthScale * roiRatio / 2.0;
  // Creating inner patch
  cv::Point2f center(x,y);
  return cv::Rect_<float>(center - cv::Point2f(half_size, 2*half_size),
                          center + cv::Point2f(half_size, 2*half_size));
}

double ObstacleByII::getPatchScore(const cv::Rect & patch,
                                   const cv::Mat & greenII) {
  // Use cropped rectangle
  cv::Rect cropped = Utils::cropRect(patch, cv::Size(cols,rows));

  // Return 0 score for empty areas
  if (cropped.area() == 0) return 0;

  // Top left and bottom right corners
  cv::Point2i tl, br;
  tl = cropped.tl();
  br = cropped.br();//Offset has to be counted on image score

  int A, B, C, D;
  A = greenII.at<int>(tl.y, tl.x);
  B = greenII.at<int>(tl.y, br.x);
  C = greenII.at<int>(br.y, tl.x);
  D = greenII.at<int>(br.y, br.x);

  double area = cropped.area();

  return (A + D - B - C) / area;
}

void ObstacleByII::fillScore(cv::Mat & img, int score,
                             int start_x, int end_x,
                             int start_y, int end_y)
{
  for (int y = start_y; y < end_y; y++) {
    for (int x = start_x; x < end_x; x++) {
      img.at<int>(y,x) = score;
    }
  }
}

}
}
