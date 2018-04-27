#pragma once

#include "Utils/Segment.hpp"

#include "rhoban_geometry/parametric_line.h"
#include <vector>
#include <map>
/**
 * Field basis starts from the center of the field.
 * axis x is directed to the opponent goal
 * axis y is directed to the top of the image
 *
 * Image basis is handled by the function fieldToImg which convert points
 * accordingly
 */

namespace Vision {
namespace Field {

class Field {
public:
  static void drawField(cv::Mat &dst,
                        const cv::Scalar &bgColor = cv::Scalar(0, 128, 0));
  // Creates the singleton if needed
  static Field *getField();

  /// Number of pixels per meter
  static double getScale(const cv::Mat &img);

  static const std::vector<Utils::Segment> &getLines();
  static const std::vector<Utils::Segment> &getGoals();
  static const std::vector<rhoban_geometry::ParametricLine> &getArenaBorders();
  static std::vector<cv::Point2f> getGoalPosts();
  static std::vector<cv::Point2f> getArenaCorners();

  static const std::map<int,cv::Point3f> & getTags();

  /**
   * postNo:
   * -1 -> post with y coordinate < 0
   *  0 -> middle of the goal
   *  1 -> post with y coordinate > 0
   */
  static cv::Point2f getGoal(int goalNo, int postNo = 0);
  static cv::Point2f getMyGoal(int postNo = 0);
  static cv::Point2f getAdvGoal(int postNo = 0);

  /**
   * Convert from field basis to img basis
   */
  static cv::Point2f fieldToImg(const cv::Mat &img, const cv::Point2f &p);
  static cv::Point2i fieldToImg(const cv::Mat &img, const cv::Point2i &p);

private:
  Field();

  void initBorders();
  void initArena();
  void initCenter();
  void initGoalZone(const cv::Point2f &init, int dir);
  void initMarks();
  void initGoals();
  void initTags();

  static int totalHeight();
  static int totalWidth();

  static int lineWidth(const cv::Mat &img);
  static int markWidth(const cv::Mat &img);
  static int centerDiameter(const cv::Mat &img);

  static Field *singleton;

  std::vector<Utils::Segment> fieldLines;
  // Center of penalty marks
  std::vector<cv::Point2f> fieldMarks;
  std::vector<Utils::Segment> goals;
  std::vector<cv::Point2f> arenaCorners;
  std::vector<rhoban_geometry::ParametricLine> arenaBorders;
  std::map<int, cv::Point3f> tags;
};
}
}
