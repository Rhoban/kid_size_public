#include "Field.hpp"

#include "Tags/CalibrationSet.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "robocup_referee/constants.h"
#include <vector>
#include <cstdlib>

#include "Utils/Interface.h"

#define IMG_WIDTH (2 * Constants::field.borderStripWidth + Constants::field.fieldLength)
#define IMG_HEIGHT (2 * Constants::field.borderStripWidth + Constants::field.fieldWidth)

#define FIELD_COLOR cv::Scalar(0, 128, 0)
#define LINES_COLOR cv::Scalar(255, 255, 255)
#define GOAL_COLOR cv::Scalar(0, 200, 200)

using namespace Vision::Utils;
using robocup_referee::Constants;
using namespace rhoban_geometry;

namespace Vision {
namespace Field {

Field *Field::singleton = NULL;

double Field::getScale(const cv::Mat &img) {
  double scaleX = img.cols / (float)totalWidth();
  double scaleY = img.rows / (float)totalHeight();
  return scaleX < scaleY ? scaleX : scaleY;
}

Field::Field() : fieldLines(), fieldMarks() {
  initBorders();
  initCenter();
  for (auto dir : {-1, 1}) {
    initGoalZone(cv::Point2f(dir * Constants::field.fieldLength / 2,
                             -Constants::field.goalAreaWidth / 2),
                 -dir);
  }
  initMarks();
  initGoals();
  initArena();
  initTags(); //FIXME: testing
}

void Field::initBorders() {
  double dx = Constants::field.fieldLength / 2;
  double dy = Constants::field.fieldWidth / 2;
  auto dirs = {-1, 1};
  for (auto &dir : dirs) {
    cv::Point2f start(dx * dir, dy * dir);
    cv::Point2f end1(dx * -dir, dy * dir);
    cv::Point2f end2(dx * dir, dy * -dir);
    fieldLines.push_back(Segment(start, end1));
    fieldLines.push_back(Segment(start, end2));
  }
}

void Field::initCenter() {
  cv::Point2f centerTop(0, -Constants::field.fieldWidth / 2);
  cv::Point2f centerBot(0, Constants::field.fieldWidth / 2);
  fieldLines.push_back(Segment(centerTop, centerBot));
}

void Field::initGoalZone(const cv::Point2f &init, int dir) {
  cv::Point2f p2 = init + cv::Point2f(dir * Constants::field.goalAreaLength, 0);
  cv::Point2f p3 = p2 + cv::Point2f(0, Constants::field.goalAreaWidth);
  cv::Point2f p4 = init + cv::Point2f(0, Constants::field.goalAreaWidth);
  fieldLines.push_back(Segment(init, p2));
  fieldLines.push_back(Segment(p2, p3));
  fieldLines.push_back(Segment(p3, p4));
  //// Lines behind goals : Outdated
  // cv::Point2f p5 = init + cv::Point2f(-dir * GOAL_DEPTH,
  //                        (Constants::field.goalAreaWidth - Constants::field.goalWidth) /
  //                        2);
  // cv::Point2f p6 = p5 + cv::Point2f(0, Constants::field.goalWidth);
  // fieldLines.push_back(Segment(p5  , p6));
}

void Field::initMarks() {
  cv::Point2f delta(Constants::field.fieldLength / 2 - Constants::field.penaltyMarkDist, 0);
  fieldMarks.push_back(cv::Point2f(0, 0));
  fieldMarks.push_back(cv::Point2f(delta));
  fieldMarks.push_back(cv::Point2f(-delta));
}

void Field::initGoals() {
  for (auto i : {-1, 1}) { // 2 Goals
    cv::Point2f center = cv::Point2f(i * Constants::field.fieldLength / 2, 0);
    cv::Point2f delta(0, Constants::field.goalWidth / 2);
    goals.push_back(Segment(center - delta, center + delta));
  }
}

void Field::initArena() {
  arenaCorners.clear();
  for (int dirX : {-1, 1}) {
    for (int dirY : {-1, 1}) {
      double distX = Constants::field.fieldLength / 2 + Constants::field.borderStripWidth;
      double distY = Constants::field.fieldWidth / 2 + Constants::field.borderStripWidth;
      arenaCorners.push_back(cv::Point2f(distX * dirX, distY * dirY));
    }
  }
  arenaBorders.clear();
  arenaBorders.push_back(ParametricLine(cv2rg(arenaCorners[0]), cv2rg(arenaCorners[1])));
  arenaBorders.push_back(ParametricLine(cv2rg(arenaCorners[0]), cv2rg(arenaCorners[2])));
  arenaBorders.push_back(ParametricLine(cv2rg(arenaCorners[1]), cv2rg(arenaCorners[3])));
  arenaBorders.push_back(ParametricLine(cv2rg(arenaCorners[2]), cv2rg(arenaCorners[3])));
}

Field *Field::getField() {
  if (singleton == NULL)
    singleton = new Field();
  return singleton;
}

cv::Point2f Field::getGoal(int goalNo, int postNo) {
  cv::Point2f offset(0, postNo * Constants::field.goalWidth / 2);
  return getField()->goals[goalNo].center() + offset;
}


void Field::initTags()
{
  tags.clear();
  CalibrationSet cs;
  std::map<int,ArucoTag> tags_dictionary = cs.getMarkers();
  for (const auto & entry : tags_dictionary) {
    Eigen::Vector3d center = entry.second.marker_center;
    tags[entry.first] = cv::Point3f(center.x(), center.y(), center.z());
  }
}

const std::map<int,cv::Point3f> & Field::getTags()
{
  return getField()->tags;
}

cv::Point2f Field::getMyGoal(int postNo) { return getGoal(0, postNo); }

cv::Point2f Field::getAdvGoal(int postNo) { return getGoal(1, postNo); }

int Field::totalHeight() {
  return (int)(2 * Constants::field.borderStripWidth + Constants::field.fieldWidth);
}

int Field::totalWidth() {
  return (int)(2 * Constants::field.borderStripWidth + Constants::field.fieldLength);
}

int Field::lineWidth(const cv::Mat &img) {
  return (int)(Constants::field.lineWidth * getScale(img));
}

int Field::markWidth(const cv::Mat &img) {
  return (int)(Constants::field.penaltyMarkLength * getScale(img));
}

int Field::centerDiameter(const cv::Mat &img) {
  return (int)(2 * Constants::field.centerRadius * getScale(img));
}

cv::Point2f Field::fieldToImg(const cv::Mat &img, const cv::Point2f &p) {
  cv::Point2f start(img.cols / 2, img.rows / 2);
  start.x += p.x * getScale(img);
  start.y -= p.y * getScale(img);
  return start;
}

cv::Point2i Field::fieldToImg(const cv::Mat &img, const cv::Point2i &p) {
  cv::Point2f start(img.cols / 2, img.rows / 2);
  start.x += p.x * getScale(img);
  start.y -= p.y * getScale(img);
  return start;
}

void Field::drawField(cv::Mat &img, const cv::Scalar &bgColor) {
  // Filling with bg Color
  rectangle(img, cv::Point(0, 0), cv::Point(img.cols, img.rows), bgColor,
            CV_FILLED);
  Field *f = getField();
  // Drawing white lines
  for (unsigned int i = 0; i < f->fieldLines.size(); i++) {
    const Segment &s = f->fieldLines[i];
    cv::Point2i src = fieldToImg(img, s.first);
    cv::Point2i dst = fieldToImg(img, s.second);
    line(img, src, dst, LINES_COLOR, lineWidth(img));
  }
  // Drawing penalty marks
  for (unsigned int i = 0; i < f->fieldMarks.size(); i++) {
    cv::Point2f markCenter = f->fieldMarks[i];
    cv::Point2f dx(Constants::field.penaltyMarkLength / 2, 0);
    cv::Point2f dy(0, Constants::field.penaltyMarkLength / 2);
    line(img, fieldToImg(img, markCenter - dx),
         fieldToImg(img, markCenter + dx), LINES_COLOR, markWidth(img));
    line(img, fieldToImg(img, markCenter - dy),
         fieldToImg(img, markCenter + dy), LINES_COLOR, markWidth(img));
  }
  // Drawing circle
  circle(img, fieldToImg(img, cv::Point(0, 0)), centerDiameter(img) / 2,
         LINES_COLOR, lineWidth(img));
  // Drawing Goals
  for (unsigned int i = 0; i < f->goals.size(); i++) {
    Segment &s = f->goals[i];
    cv::Point2f src = s.first;
    cv::Point2f dst = s.second;
    line(img, fieldToImg(img, src), fieldToImg(img, dst), GOAL_COLOR,
         3 * lineWidth(img));
  }

  // drawing goal posts
  circle(img, fieldToImg(img, getAdvGoal(-1)), 2, cv::Scalar(0, 0, 255), 2);
  circle(img, fieldToImg(img, getAdvGoal(1)), 2, cv::Scalar(0, 0, 255), 2);

  circle(img, fieldToImg(img, getMyGoal(-1)), 2, cv::Scalar(0, 0, 255), 2);
  circle(img, fieldToImg(img, getMyGoal(1)), 2, cv::Scalar(0, 0, 255), 2);
}

const std::vector<Segment> &Field::getLines() { return getField()->fieldLines; }

const std::vector<Segment> &Field::getGoals() { return getField()->goals; }

const std::vector<ParametricLine> &Field::getArenaBorders() {
  return getField()->arenaBorders;
}

std::vector<cv::Point2f> Field::getGoalPosts() {
  std::vector<cv::Point2f> goalPosts;
  std::vector<Segment> goals = getGoals();
  for (unsigned int i = 0; i < goals.size(); i++) {
    goalPosts.push_back(goals[i].first);
    goalPosts.push_back(goals[i].second);
  }
  return goalPosts;
}

std::vector<cv::Point2f> Field::getArenaCorners() {
  return getField()->arenaCorners;
  std::vector<cv::Point2f> corners;
  for (int dirX : {-1, 1}) {
    for (int dirY : {-1, 1}) {
      double distX = Constants::field.fieldLength / 2 + Constants::field.borderStripWidth;
      double distY = Constants::field.fieldWidth / 2 + Constants::field.borderStripWidth;
      corners.push_back(cv::Point2f(distX * dirX, distY * dirY));
    }
  }
  return corners;
}
}
}
