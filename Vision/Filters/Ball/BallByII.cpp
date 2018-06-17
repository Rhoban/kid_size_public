#include "BallByII.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rhoban_utils/timing/benchmark.h"
#include "Utils/OpencvUtils.h"
#include "Utils/ROITools.hpp"
#include "Utils/RotatedRectUtils.hpp"
#include "CameraState/CameraState.hpp"

#include <set>

using rhoban_utils::Benchmark;

namespace Vision {
namespace Filters {

std::string BallByII::getClassName() const {
  return "BallByII";
}

int BallByII::expectedDependencies() const {
  return 3;
}


void BallByII::setParameters() {
  boundaryFactor = ParamFloat(2.0,1.0,3.0);
  maxBoundaryThickness = ParamFloat(30.0,1.0,100.0);
  minRadius = ParamFloat(2.0,0.5,20.0);
  minScore = ParamFloat(100.0,0.0,510.);
  boundaryFactor = ParamFloat(2.0,1.0,3.0);
  yWeight = ParamFloat(10.0,0.0,20.0);
  greenWeight = ParamFloat(1.0,0.0,20.0);
  maxRois = ParamInt(4,1,100);
  decimationRate = ParamInt(1,1,20);
  tagLevel = ParamInt(0,0,1);
  useLocalSearch = ParamInt(1,0,1);

  params()->define<ParamFloat>("boundaryFactor", &boundaryFactor);
  params()->define<ParamFloat>("maxBoundaryThickness", &maxBoundaryThickness);
  params()->define<ParamFloat>("yWeight", &yWeight);
  params()->define<ParamFloat>("greenWeight", &greenWeight);
  params()->define<ParamFloat>("minRadius", &minRadius);
  params()->define<ParamFloat>("minScore", &minScore);
  params()->define<ParamInt>("maxRois", &maxRois);
  params()->define<ParamInt>("decimationRate", &decimationRate);
  params()->define<ParamInt>("tagLevel", &tagLevel);
  params()->define<ParamInt>("useLocalSearch", &useLocalSearch);
}

void BallByII::process() {
  // Get names of dependencies
  const std::string & yName = _dependencies[0];
  const std::string & greenName = _dependencies[1];
  const std::string & radiusProviderName = _dependencies[2];
  // Import source matrix and update size
  const cv::Mat & yImg = *(getDependency(yName).getImg());
  const cv::Mat & greenImg = *(getDependency(greenName).getImg());
  const cv::Mat & radiusImg = *(getDependency(radiusProviderName).getImg());

  // Integral images have 1 line and 1 column more than resulting image
  rows = yImg.rows - 1;
  cols = yImg.cols - 1;

  //TODO: checks on size consistency

  clearRois();

  std::vector<std::pair<double, cv::Rect>> tmpRois;

  // minScore <= 0, maxScore >= 0
  double imgMinScore = 0;
  double imgMaxScore = 0;

  const cv::Size & size = yImg.size();

  Benchmark::open("computing decimated scores");

  // Computing score matrix and ROI at once
  cv::Mat scores;
  if (tagLevel > 0) {
    scores = cv::Mat(rows, cols, CV_32SC1, cv::Scalar(0.0));
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

      // Getting the middle point
      int center_x = (start_x + end_x) / 2;
      int center_y = (start_y + end_y) / 2; 

      // Computing boundary patch
      float radius = radiusImg.at<float>(center_y,center_x);
      cv::Rect_<float> boundaryPatch = getBoundaryPatch(center_x,center_y, radius);

      // If area is not entirely inside the image or expected radius is too small:
      // - Skip ROI and use a '0' score
      if (!Utils::isContained(boundaryPatch, size) ||
          radius <= minRadius) {
        if (tagLevel > 0) {
          fillScore(scores, 0, start_x, end_x, start_y, end_y);
        }
        continue;
      }

      double score = getCandidateScore(center_x, center_y, radius, yImg, greenImg);

      // Write score in scores map
      if (tagLevel > 0) {
        fillScore(scores, (int)score, start_x, end_x, start_y, end_y);
        // Update score boundaries
        if (score > imgMaxScore) {
          imgMaxScore = score;
        }
        if (score < imgMinScore) {
          imgMinScore = score;
        }
      }

      // If score is too low to use it, skip
      if (score < minScore) {
        continue;
      }

      // Establishing:
      // - List of dominated ROIs
      // - Worst score
      // If current candidate is dominated, then stop the process and ignore current candidate
      std::set<size_t> dominated_rois;
      bool dominated = false;
      double worstScore = score;
      int worstId = -1;
      const cv::Rect & candidatePatch = boundaryPatch;
      for (size_t id = 0; id < tmpRois.size(); id++) {
        double roiScore = tmpRois[id].first;
        const cv::Rect & roi = tmpRois[id].second;
        // If there is an overlap with 'id'
        if (Utils::isOverlapping(roi, candidatePatch)) {
          // If new region is better, then it dominates 'id'
          if (roiScore < score) {
            dominated_rois.insert(id);
          }
          // If new region is not better, simply stop the process and ignore the region
          else {
            dominated = true;
            break;
          }
        }
        // If score is lower than previously met, update
        if (worstScore > roiScore) {
          worstScore = roiScore;
          worstId = id;
        }
      }
      // If roi is dominated, jump to next ROI
      if (dominated) {
      }
      // No areas directly dominated
      else if (dominated_rois.size() == 0) {
        // If there is enough space remaining, push element
        if ((int)tmpRois.size() < maxRois) {
          tmpRois.push_back({score, candidatePatch});
        }
        // If there is not enough space: replace worst ROI (if patch is better)
        else if (worstId != -1) {
          tmpRois[worstId] = {score, candidatePatch};
        }
      }
      // If one area is dominated, replace it
      else if (dominated_rois.size() == 1) {
        tmpRois[*(dominated_rois.begin())] = {score, candidatePatch};
      }
      // If more than one area is dominated, remove all dominated areas and add current one
      else {
        std::vector<std::pair<double, cv::Rect>> oldRois = tmpRois;
        tmpRois.clear();
        for (size_t roi_idx = 0; roi_idx < oldRois.size(); roi_idx++) {
          bool discard = dominated_rois.count(roi_idx) == 1;
          if (!discard) {
            tmpRois.push_back(oldRois[roi_idx]);
          }
        }
        tmpRois.push_back({score, candidatePatch});
      }
    }
  }

  Benchmark::close("computing decimated scores");

  if (useLocalSearch) {
    Benchmark::open("local search");
    std::vector<std::pair<double, cv::Rect>> locallySearchedRois;
    for (const auto & roi : tmpRois) {
      cv::Rect initial_patch = roi.second;
      int center_x = initial_patch.x + initial_patch.width / 2;
      int center_y = initial_patch.y + initial_patch.height / 2;
      
      double bestScore = minScore;
      cv::Rect bestPatch;
      int maxOffset = ceil(decimationRate /2.0);
      for (int xOffset = -maxOffset; xOffset <= maxOffset; xOffset++) {
        for (int yOffset = -maxOffset; yOffset <= maxOffset; yOffset++) {
          int new_center_x = center_x + xOffset;
          int new_center_y = center_y + yOffset;
          float radius = radiusImg.at<float>(new_center_y,new_center_x);
          cv::Rect newPatch = getBoundaryPatch(new_center_x,new_center_y, radius);
          if (!Utils::isContained(newPatch, size)) continue;
          double score = getCandidateScore(new_center_x, new_center_y, radius, yImg, greenImg);
          if (score > bestScore) {
            bestScore = score;
            bestPatch = newPatch;
          }
        }
      }
      locallySearchedRois.push_back({bestScore, bestPatch});
    }
    tmpRois = locallySearchedRois;
    Benchmark::close("local search");
  }
  
  for (const std::pair<double, cv::Rect> & scored_roi : tmpRois) {
    addRoi(scored_roi.first, Utils::toRotatedRect(scored_roi.second));
  }

  Benchmark::open("getHeatMap");
  if (tagLevel > 0) {
    img() = getHeatMap(scores, imgMinScore, imgMaxScore);
  }
  else {
    img() = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
  }
  Benchmark::close("getHeatMap");
}

cv::Mat BallByII::getHeatMap(const cv::Mat & scores,
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

double BallByII::getBoundaryHalfWidth(float radius)
{
  return std::min(radius * boundaryFactor, radius + maxBoundaryThickness);
}

cv::Rect_<float> BallByII::getInnerPatch(int x, int y, float radius)
{
  // Creating inner patch
  cv::Point2f center(x,y);
  cv::Point2f halfSize(radius, radius);
  return cv::Rect_<float>(center - halfSize, center + halfSize);
}

cv::Rect_<float> BallByII::getInnerAbovePatch(int x, int y, float radius)
{
  cv::Point2f center(x,y);
  cv::Point2f halfSize(radius, radius);
  return cv::Rect_<float>(center - halfSize, center + cv::Point2f(radius,0));
}

cv::Rect_<float> BallByII::getInnerBelowPatch(int x, int y, float radius)
{
  cv::Point2f center(x,y);
  cv::Point2f halfSize(radius, radius);
  return cv::Rect_<float>(center - cv::Point2f(radius,0), center + halfSize);
}

cv::Rect_<float> BallByII::getBoundaryPatch(int x, int y, float radius)
{
  // Creating boundary patch
  cv::Point2f center(x,y);
  double halfWidth = getBoundaryHalfWidth(radius);
  cv::Point2f halfSize(halfWidth, halfWidth);
  return cv::Rect_<float>(center - halfSize, center + halfSize);
}

cv::Rect_<float> BallByII::getBoundaryAbovePatch(int x, int y, float radius)
{
  double halfWidth = getBoundaryHalfWidth(radius);
  cv::Point2f tl(x-radius,y-halfWidth);
  cv::Point2f fullSize(2*radius, halfWidth - radius);
  return cv::Rect_<float>(tl, tl + fullSize);
}

double BallByII::getPatchScore(const cv::Rect & patch,
                               const cv::Mat & integralImage) {
  // Use cropped rectangle
  cv::Rect cropped = Utils::cropRect(patch, cv::Size(cols,rows));

  // Return null score if cropped area is small
  if (cropped.area() == 0) return 0;

  // Top left and bottom right corners
  cv::Point2i tl, br;
  tl = cropped.tl();
  br = cropped.br();//Offset has to be counted on image score

  int A, B, C, D;
  A = integralImage.at<int>(tl.y, tl.x);
  B = integralImage.at<int>(tl.y, br.x);
  C = integralImage.at<int>(br.y, tl.x);
  D = integralImage.at<int>(br.y, br.x);

  double area = cropped.area();

  return (A + D - B - C) / area;
}

double BallByII::getCandidateScore(int center_x, int center_y, double radius,
                                   const cv::Mat & yImg, const cv::Mat & greenImg) {
  // Computing inner patches
  cv::Rect_<float> boundaryPatch = getBoundaryPatch(center_x, center_y, radius);
  cv::Rect_<float> innerPatch = getInnerPatch(center_x, center_y, radius);
  cv::Rect_<float> innerAbovePatch = getInnerAbovePatch(center_x, center_y, radius);
  cv::Rect_<float> innerBelowPatch = getInnerBelowPatch(center_x, center_y, radius);
  cv::Rect_<float> boundaryAbovePatch = getBoundaryAbovePatch(center_x, center_y, radius);

  // Abreviations are the following:
  // ia: inner above
  // ib: inner below
  // b : boundary
  // ub: upper boundary

  // Computing y_score
  double y_ia = getPatchScore(innerAbovePatch, yImg);
  double y_ib = getPatchScore(innerBelowPatch, yImg);
  double y_b  = getPatchScore(boundaryPatch, yImg);
  double y_ub = getPatchScore(boundaryAbovePatch, yImg);
  // When ball is far: IA is supposed to be bright and the rest supposed to be dark
  // score_far = (y_ia - y_ub) + (y_ia - y_ib) + (y_ia - y_b)
  // (y_ia - y_ub) -> Avoid white object on the top of the boundary zone
  // (y_ia - y_ib) -> Upper part has to be bright
  // (y_ia - y_b ) -> Difference with the global boundary
  double y_score_far = 3 * y_ia - y_ub - y_ib - y_b;
  double y_score_close = y_ia + y_ib - 2 * y_b;
  double y_score = std::max(y_score_far, y_score_close);

  // Computing green_score
  double green_b = getPatchScore(boundaryPatch, greenImg);
  double green_i = getPatchScore(innerPatch, greenImg);
  double green_score = green_b - green_i;

  return (yWeight * y_score + greenWeight * green_score) / (yWeight + greenWeight);
}

void BallByII::fillScore(cv::Mat & img, int score,
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
