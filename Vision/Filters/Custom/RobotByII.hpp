#pragma once

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/// Uses an algorithm based on the approach proposed by Berlin United (spl team)
/// at RoHOW 2017 to identify the best 'n' regions of interest
///
/// Input:
/// - WhiteII: An integral image of (rows + 1, cols + 1) monochannel and at least 32 bits
/// - GreenII: An integral image of (rows + 1, cols + 1) monochannel and at least 32 bits
/// - DarkII: An integral image of (rows + 1, cols + 1) monochannel and at least 32 bits
/// - GoalWidth: An image of (rows, cols) monochannel (CV_32F)
/// Output:
/// - An image of (rows, cols) where pixels are colored in blue or red depending on their score
///
/// Three different types of patches are considered
/// - Above (width approximately of the robot)
/// - Below (width approximately of the robot)
/// - Boundary (top is top of above, bottom is bottom of below, width is larger)
class RobotByII : public Filter {
public:
  RobotByII();

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;

  // Image position and score
  std::vector<std::pair<cv::Point2f, float> > robots;

protected:
  virtual void process() override;

  virtual void setParameters() override;

  cv::Mat getHeatMap(const cv::Mat & scores, double minScore, double maxScore) const;
  
private:
  /// Return the patch associated to the above rect from the given point
  cv::Rect_<float> getAbovePatch(int x, int y, float width);

  /// Return the patch associated to the above right rect from the given point
  cv::Rect_<float> getAboveRightPatch(int x, int y, float width);

  /// Return the patch associated to the above left rect from the given point
  cv::Rect_<float> getAboveLeftPatch(int x, int y, float width);

  /// Return the patch associated to the below rect from given point
  cv::Rect_<float> getBelowPatch(int x, int y, float width);

  /// Return the patch associated to the boundary rect at the given point
  cv::Rect_<float> getBoundaryPatch(int x, int y, float width);

  /// Return the patch associated to the roi rect at the given point
  cv::Rect_<float> getROIPatch(int x, int y, float width);

  /// Return the score of the patch given the provided integralImage
  double getPatchScore(const cv::Rect& patch, const cv::Mat& whiteII, const cv::Mat& greenII, const cv::Mat& darkII);

  /// img should be CV_32SC1
  /// draw pixels in [start_x,end_x[ * [start_y,end_y[
  void fillScore(cv::Mat & img, int score,
                 int start_x, int end_x,
                 int start_y, int end_y);

  void drawChosenOnes(cv::Mat &img, std::vector<std::pair<cv::Point2f, float> > robots, int cols, int rows, int mode=1);
  
  /// Sorting a list of robots based on how close they are to the bottom of the image.
  void sortRobots(std::vector<std::pair<cv::Point2f, float> > & robots);
  std::vector<std::pair<cv::Point2f, float> > shadowRobots(std::vector<std::pair<cv::Point2f, float> > & robots);
  /// Robots that are considered near the center are destroyed. "Near" is defined as a rectangle
  /// of the expected actualRobotWidth and actualRobotHeight (m).
  int destroyNear(cv::Point2f center, std::vector<std::pair<cv::Point2f, float> > & robots);
  bool isNear(cv::Point2f center, cv::Point2f p);

  /// Starts at (x, y) and looks for a point whose green density or white density is high enough, in
  /// the direction xDir, yDir.
  /// The aformentioned point is given through result.
  float findLimit(cv::Point2i *result, cv::Mat &green, cv::Mat &white, int threshGreen, int threshWhite, int x, int y,
                  int xDir, int yDir);
  
  /// Uses the above methods to find a unique point per robot (ideally just below the robot feet)
  void filterCandidates(std::vector<std::pair<cv::Point2f, float> > & robots, cv::Mat & greenDensity, cv::Mat & whiteDensity);

  /// Decides if a point (in the robot frame) is so close that it's probably ourself.
  bool isItMyself(cv::Point2f p);



  /// Number of rows in the result image
  int rows;

  /// Number of columns in the result image;
  int cols;

  /// A ratio used to increase width size
  ParamFloat widthScale;

  /// Inner patch above point: aboveRatio * width * widthScale
  ParamFloat aboveRatio;

  /// Boundary patch below point: belowRatio * width * widthScale
  ParamFloat belowRatio;

  /// Boundary width: boundaryWidthRatio * width * widthScale
  ParamFloat boundaryWidthRatio;

  /// Size of the ROI: roiRatio * width * widthScale (squared, centered)
  ParamFloat roiRatio;

  /// Minimal expected width (after widthScale) to accept the region
  ParamFloat minWidth;

  /// (darkMatters*darkScore + 1*exteriorScore) / darkMatters+1 
  ParamFloat darkMatters;
  
  /// Minimal score for considering that region of interest is acceptable
  /// Reminder: score is in [-510,510]
  ParamFloat minScore;

  /// expected robotWidth in m.
  ParamFloat robotWidth;

  /// expected robotWidth in m (but for real this time, used in the filtering section to delete points
  /// that are inside the same robot).
  ParamFloat actualRobotWidth;
  ParamFloat actualRobotHeight;
  
  /// Maximal number of ROI generated by the filter
  ParamInt maxRois;

  // What is the weight of (above - below)
  ParamFloat belowCoeff;
  // What is the weight of (2*above - above_right - above_left)
  ParamFloat sideCoeff;

  ParamFloat greenThreshold;
  ParamFloat whiteThreshold;

  // in m
  ParamFloat tooCloseToBeTrue;
  
  /// Since using a high resolution Integral Image helps to detect balls which
  /// are far away, a decimationRate is used to compute the score function only
  /// at an interval given by decimationRate
  ParamInt decimationRate;

  /// 0: No heatMap produced
  ParamInt tagLevel;
};
}
}


