#include "Field.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "robocup_referee/constants.h"
#include <vector>
#include <cstdlib>

#include "Utils/Interface.h"

#define IMG_WIDTH (2 * Constants::borderStrip + Constants::field.fieldLength)
#define IMG_HEIGHT (2 * Constants::borderStrip + Constants::field.fieldWidth)

#define FIELD_COLOR cv::Scalar(0, 128, 0)
#define LINES_COLOR cv::Scalar(255, 255, 255)
#define GOAL_COLOR cv::Scalar(0, 200, 200)

using namespace Vision::Utils;
using robocup_referee::Constants;
using namespace rhoban_geometry;

namespace Vision {
namespace Field {

Field *Field::singleton = NULL;

cv::Point Field::offset() { return cv::Point(borderStrip, borderStrip); }

const double Field::centerRadius = Constants::field.centerRadius;
const double Field::fieldLength = Constants::field.fieldLength;
const double Field::fieldWidth = Constants::field.fieldWidth;
const double Field::goalWidth = Constants::field.goalWidth;
const double Field::goalAreaLength = Constants::field.goalAreaLength;
const double Field::goalAreaWidth = Constants::field.goalAreaWidth;
const double Field::penaltyMarkDist = Constants::field.penaltyMarkDist;
const double Field::borderStrip = Constants::field.borderStripWidth;

double Field::getScale(const cv::Mat &img) {
  double scaleX = img.cols / (float)totalWidth();
  double scaleY = img.rows / (float)totalHeight();
  return scaleX < scaleY ? scaleX : scaleY;
}

Field::Field() : fieldLines(), fieldMarks() {
  initBorders();
  initCenter();
  for (auto dir : {-1, 1}) {
    initGoalZone(cv::Point(dir * fieldLength / 2, -goalAreaWidth / 2), -dir);
  }
  initMarks();
  initGoals();
  initArena();
  initTags(); //FIXME: testing
}

void Field::initBorders() {
  int dx = fieldLength / 2;
  int dy = fieldWidth / 2;
  auto dirs = {-1, 1};
  for (auto &dir : dirs) {
    cv::Point start(dx * dir, dy * dir);
    cv::Point end1(dx * -dir, dy * dir);
    cv::Point end2(dx * dir, dy * -dir);
    fieldLines.push_back(Segment(start, end1));
    fieldLines.push_back(Segment(start, end2));
  }
}

void Field::initCenter() {
  cv::Point centerTop(0, -fieldWidth / 2);
  cv::Point centerBot(0, fieldWidth / 2);
  fieldLines.push_back(Segment(centerTop, centerBot));
}

void Field::initGoalZone(const cv::Point &init, int dir) {
  cv::Point p2 = init + cv::Point(dir * goalAreaLength, 0);
  cv::Point p3 = p2 + cv::Point(0, goalAreaWidth);
  cv::Point p4 = init + cv::Point(0, goalAreaWidth);
  fieldLines.push_back(Segment(init, p2));
  fieldLines.push_back(Segment(p2, p3));
  fieldLines.push_back(Segment(p3, p4));
  //// Lines behind goals : Outdated
  // cv::Point p5 = init + cv::Point(-dir * GOAL_DEPTH,
  //                        (Constants::field.goalAreaWidth - Constants::field.goalWidth) /
  //                        2);
  // cv::Point p6 = p5 + cv::Point(0, Constants::field.goalWidth);
  // fieldLines.push_back(Segment(p5  , p6));
}

void Field::initMarks() {
  cv::Point delta(fieldLength / 2 - penaltyMarkDist, 0);
  fieldMarks.push_back(cv::Point(0, 0));
  fieldMarks.push_back(cv::Point(delta));
  fieldMarks.push_back(cv::Point(-delta));
}

void Field::initGoals() {
  for (auto i : {-1, 1}) { // 2 Goals
    cv::Point center = cv::Point(i * fieldLength / 2, 0);
    cv::Point delta(0, goalWidth / 2);
    goals.push_back(Segment(center - delta, center + delta));
  }
}

void Field::initArena() {
  arenaCorners.clear();
  for (int dirX : {-1, 1}) {
    for (int dirY : {-1, 1}) {
      double distX = fieldLength / 2 + borderStrip;
      double distY = fieldWidth / 2 + borderStrip;
      arenaCorners.push_back(cv::Point(distX * dirX, distY * dirY));
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

cv::Point Field::getGoal(int goalNo, int postNo) {
  cv::Point offset(0, postNo * goalWidth / 2);
  return getField()->goals[goalNo].center() + offset;
}


void Field::initTags()
{
  double tagHeight = 15;// cm
  /**
   *  CONFIG:                                 <- Middle of the goal
   *  141 -------------------------------------
   *  |               |                       
   *  |               |                       
   *  |              173-----------------------
   *  |                                        
   *  |                                        
   * 132                                        
   *  |                                      29 <- Penalty mark
   *  |                                        
   *  |                                   -------
   *  |                                  /       \
   *  |                                 /         \
   *  67-----------------62------------------55----
   *                     /\
   *              Quarter of the field
   */
  cv::Point3f penaltyMark(fieldLength / 2 - penaltyMarkDist, 0, tagHeight);
  cv::Point3f goalAreaCorner1(fieldLength / 2 - goalAreaLength, goalAreaWidth/2, tagHeight);
  tags[ 29] = penaltyMark;
  tags[ 55] = cv::Point3f(0,0, tagHeight);
  tags[ 62] = cv::Point3f(0, fieldWidth/4, tagHeight);
  tags[ 67] = cv::Point3f(0, fieldWidth/2, tagHeight);
  tags[132] = cv::Point3f(fieldLength/4, fieldWidth/2, tagHeight);
  tags[141] = cv::Point3f(fieldLength/2, fieldWidth/2, tagHeight);
  tags[173] = goalAreaCorner1;
}

const std::map<int,cv::Point3f> & Field::getTags()
{
  return getField()->tags;
}

cv::Point Field::getMyGoal(int postNo) { return getGoal(0, postNo); }

cv::Point Field::getAdvGoal(int postNo) { return getGoal(1, postNo); }

int Field::totalHeight() { return (int)(2 * borderStrip + fieldWidth); }

int Field::totalWidth() { return (int)(2 * borderStrip + fieldLength); }

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
    cv::Point src = fieldToImg(img, s.first);
    cv::Point dst = fieldToImg(img, s.second);
    line(img, src, dst, LINES_COLOR, lineWidth(img));
  }
  // Drawing penalty marks
  for (unsigned int i = 0; i < f->fieldMarks.size(); i++) {
    cv::Point markCenter = f->fieldMarks[i];
    cv::Point dx(markWidth(img) / 2, 0);
    cv::Point dy(0, markWidth(img) / 2);
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
    cv::Point src = s.first;
    cv::Point dst = s.second;
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

std::vector<cv::Point> Field::getGoalPosts() {
  std::vector<cv::Point> goalPosts;
  std::vector<Segment> goals = getGoals();
  for (unsigned int i = 0; i < goals.size(); i++) {
    goalPosts.push_back(goals[i].first);
    goalPosts.push_back(goals[i].second);
  }
  return goalPosts;
}

std::vector<cv::Point> Field::getArenaCorners() {
  return getField()->arenaCorners;
  std::vector<cv::Point> corners;
  for (int dirX : {-1, 1}) {
    for (int dirY : {-1, 1}) {
      double distX = fieldLength / 2 + borderStrip;
      double distY = fieldWidth / 2 + borderStrip;
      corners.push_back(cv::Point(distX * dirX, distY * dirY));
    }
  }
  return corners;
}
}
}
