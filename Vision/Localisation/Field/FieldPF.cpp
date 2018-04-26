#include <iostream>

#include "FieldPF.hpp"

#include "Field/Field.hpp"
#include "../RobotBasis.hpp"

#include "rhoban_utils/logging/logger.h"

#include <robocup_referee/constants.h>
#include <vector>
#include <map>
#include <sstream>
#include <cmath>

static rhoban_utils::Logger logger("FieldPF");

using namespace std;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace rhoban_unsorted;
using namespace robocup_referee;

namespace Vision {
namespace Localisation {

std::map<FieldPF::ResetType,std::string> FieldPF::resetNames =
{
  {ResetType::None, "None"},
  {ResetType::Uniform, "Uniform"},
  {ResetType::BordersRight, "BordersRight"},
  {ResetType::BordersLeft, "BordersLeft"},
  {ResetType::Borders, "Borders"},
  {ResetType::Fall, "Fall"},
  {ResetType::Custom, "Custom"}
};

double FieldPF::mins[3] = {-Constants::field.fieldLength / 200,
                           -Constants::field.fieldWidth / 200, 0};
double FieldPF::maxs[3] = {Constants::field.fieldLength / 200,
                           Constants::field.fieldWidth / 200, 360};

FieldPF::FieldPF()
    : ParticleFilter(), resetType(ResetType::None),
      errorTols({8}), resamplingRatio(0.0), tolDist(100),
      tolDiffAngle(15) {
  std::vector<int> goalsToTrack = {1};
  std::vector<int> postsToTrack = {-1, 0, 1};
  for (int goal : goalsToTrack) {
    for (int post : postsToTrack) {
      GoalKey gk(goal, post);
      trackedGoals.push_back(gk);
      goalDirections[gk] = Angle(0);
      for (int errorTol : errorTols) {
        QualityKey qk(goal, post, errorTol);
        goalQualities[qk] = 0;
      }
    }
  }
  RhIO::Root.newChild("/Localisation/Field/FieldPF");
  rhioNode = &(RhIO::Root.child("/Localisation/Field/FieldPF"));
  rhioNode->newFloat("resamplingRatio")
      ->defaultValue(0)
      ->minimum(0)
      ->maximum(1)
      ->comment("Ratio of particles resampled");
  rhioNode->newFloat("goalDir")->defaultValue(0);
  rhioNode->newFloat("goalDirQ")->defaultValue(0)->minimum(0)->maximum(1);
  rhioNode->newFloat("leftGoalDir")->defaultValue(0);
  rhioNode->newFloat("rightGoalDir")->defaultValue(0);
  rhioNode->newFloat("posX")->defaultValue(0);
  rhioNode->newFloat("posY")->defaultValue(0);
  rhioNode->newFloat("azimuth")->defaultValue(0);
  rhioNode->newFloat("posQ")->defaultValue(0)->minimum(0)->maximum(1);
  rhioNode->newFloat("tolDist")
      ->defaultValue(tolDist)
      ->minimum(0)
      ->maximum(600)
      ->persisted(true);
  rhioNode->newFloat("tolDiffAngle")
      ->defaultValue(tolDiffAngle)
      ->minimum(0)
      ->maximum(180)
      ->persisted(true);
  rhioNode->newFloat("fallNoise")
      ->defaultValue(25)
      ->minimum(0)
      ->maximum(200)
      ->comment("Maximal noise applied when falling [cm]")
      ->persisted(true);
  rhioNode->newFloat("fallNoiseTheta")
      ->defaultValue(20)
      ->minimum(0)
      ->maximum(180)
      ->comment("When falling, uniform noise in [-x,x] is applied [deg]")
      ->persisted(true);
  rhioNode->newFloat("borderNoise")
      ->defaultValue(50)
      ->minimum(0)
      ->maximum(200)
      ->comment(
            "Maximal noise applied in 'x' on the position of the robot [cm]")
      ->persisted(true);
  rhioNode->newFloat("borderNoiseTheta")
      ->defaultValue(10)
      ->minimum(0)
      ->maximum(180)
      ->comment("Orientation noise applied at border reset in [-x,x] [deg]")
      ->persisted(true);
  rhioNode->newFloat("customX")
      ->defaultValue(0)
      ->minimum(-Constants::field.fieldLength / 100)
      ->maximum(Constants::field.fieldLength / 100)
      ->comment("Position used for customReset [cm]");
  rhioNode->newFloat("customY")
      ->defaultValue(0)
      ->minimum(-Constants::field.fieldWidth / 100)
      ->maximum(Constants::field.fieldWidth / 100)
      ->comment("Position used for customReset [cm]");
  rhioNode->newFloat("customTheta")
      ->defaultValue(0)
      ->minimum(-180)
      ->maximum(180)
      ->comment("Theta used for customReset (deg)");
  rhioNode->newFloat("customThetaNoise")
      ->defaultValue(10)
      ->minimum(0)
      ->maximum(180)
      ->comment("Uniform noise used for custom reset [deg]");
  rhioNode->newFloat("customNoise")
      ->defaultValue(50)
      ->minimum(0)
      ->maximum(400)
      ->comment("Uniform noise used for custom reset [cm]");
}

void FieldPF::updateToGoalQuality() {
  // Initializing data for average computation
  std::map<GoalKey, std::vector<Angle>> viewedAngles;
  for (const auto &goalPost : trackedGoals) {
    viewedAngles[goalPost] = {};
  }
  // Calculating differences
  for (const auto &p : particles) {
    Point tmp = p.first.getRobotPosition();
    for (const auto &gk : trackedGoals) {
      cv::Point dst = Field::Field::getGoal(gk.first, gk.second);
      cv::Point2f pos(tmp.getX(), tmp.getY());
      // Angle in robot referential = aToGoal - robotAngle
      Angle aToGoal = Angle::fromXY(dst.x - pos.x, dst.y - pos.y);
      Angle viewedAngle = aToGoal - p.first.getOrientation();
      viewedAngles[gk].push_back(viewedAngle);
    }
  }
  // Calculating average and qualities
  for (const auto &gk : trackedGoals) {
    Angle average = Angle::mean(viewedAngles[gk]);
    std::map<int, int> goodParticulesByTol;
    for (int errorTol : errorTols) {
      goodParticulesByTol[errorTol] = 0;
    }
    // Counting particules around average
    for (const auto &viewedAngle : viewedAngles[gk]) {
      double aDiff = fabs((average - viewedAngle).getSignedValue());
      for (int errorTol : errorTols) {
        if (aDiff < errorTol) {
          goodParticulesByTol[errorTol]++;
        }
      }
    }
    // Apply calculated values
    for (int errorTol : errorTols) {
      QualityKey qk(gk.first, gk.second, errorTol);
      goalQualities[qk] =
          (float)goodParticulesByTol[errorTol] / (float)particles.size();
    }
    goalDirections[gk] = average;
  }
}

void FieldPF::askForReset(ResetType t)
{
  ResetType previousType;
  resetMutex.lock();
  previousType = resetType;
  resetType = t;
  resetMutex.unlock();
  std::string new_name = resetNames.at(t);
  std::string old_name = resetNames.at(previousType);
  logger.log("asking for a reset of type: '%s'", new_name.c_str());
  if (previousType != ResetType::None) {
    logger.warning("Overwriting a reset of type: '%s' with a reset of type '%s'",
                   old_name.c_str(), new_name.c_str());
  }
}

void FieldPF::cancelPendingReset(ResetType t)
{
  ResetType previousType;
  resetMutex.lock();
  previousType = resetType;
  if (previousType == t) {
    resetType = ResetType::None;
  }
  resetMutex.unlock();
  if (previousType == t) {
    std::string previous_name = resetNames.at(t);
    logger.log("A reset of type '%s' has been canceled",previous_name.c_str());
  }
}

FieldPF::ResetType FieldPF::getPendingReset()
{
  ResetType result;
  resetMutex.lock();
  result = resetType;
  resetMutex.unlock();
  return result;
}

void FieldPF::step(
    Controller<FieldPosition> &ctrl,
    const std::vector<Observation<FieldPosition> *> &observations,
    double elapsedTime)
{
  ResetType stepReset;
  resetMutex.lock();
  stepReset = resetType;
  resetType = ResetType::None;
  resetMutex.unlock();

  // If we are doing a reset, make sure we are up to date with rhio values
  if (stepReset != None) {
    importFromRhIO();
    std::string reset_name = resetNames.at(stepReset);
    logger.log("Applying a reset of type: '%s'", reset_name.c_str());
  }

  switch (stepReset) {
  case None: {
    // If no reset is planned, apply observations, resampling if required and return
    ParticleFilter::step(ctrl, observations, elapsedTime);
    partialUniformResampling(resamplingRatio, mins, maxs);
    return;
  }
  case Uniform: {
    initializeAtUniformRandom(particles.size());
    break;
  }
  case BordersRight:
    resetOnLines(-1);
    break;
  case BordersLeft:
    resetOnLines(1);
    break;
  case Borders:
    resetOnLines(0);
    break;
  case Fall: {
    fallReset();
    break;
  }
  case Custom: {
    customReset();
    break;
  }
  }

  // After a reset, we apply odometry but not observations
  // (robot might have moved between request of reset and now)
  ParticleFilter::step(ctrl, elapsedTime);

  // After a reset, update internal values
  updateInternalValues();
}

void FieldPF::resetOnLines(int side) {
  auto generator = rhoban_random::getRandomEngine();
  // According to rules, robot start in its own half
  double xOffset = Constants::field.penaltyMarkDist - Constants::field.fieldLength / 2;
  std::uniform_real_distribution<double> xDistribution(-borderNoise,
                                                       borderNoise);
  std::uniform_real_distribution<double> dirNoiseDistribution(-borderNoiseTheta,
                                                              borderNoiseTheta);
  std::uniform_int_distribution<int> sideDistribution(0,1);
  for (auto &p : particles) {
    double x = xOffset + xDistribution(generator);
    int currSide;
    // 0 * 2 - 1 = -1 and 1 * 2 - 1 = 1
    if (side > 0) {
      currSide = 1;
    }
    else if (side < 0) {
      currSide = -1;
    }
    // Side is unknown: sampling
    else {
      currSide = sideDistribution(engine) == 0 ? 1 : -1;
    }
    double y = currSide * Constants::field.fieldWidth / 2;
    double dirNoise = dirNoiseDistribution(generator);
    double dir = -currSide * 90;
    p.first = FieldPosition(x, y, Angle(dir + dirNoise).getSignedValue());
  }
  logger.log("Reset particles on borders");
}

void FieldPF::fallReset() {
  auto generator = rhoban_random::getRandomEngine();
  std::uniform_real_distribution<double> dirDistribution(-fallNoiseTheta,
                                                         fallNoiseTheta);
  for (auto &p : particles) {
    Point move = Point::mkRandomPolar(fallNoise);
    p.first.move(move);
    p.first.rotate(dirDistribution(generator));
  }
  logger.log("Add noise on particles due to a fall");
}

void FieldPF::customReset() {
  auto generator = rhoban_random::getRandomEngine();
  std::uniform_real_distribution<double> dirDistribution(
      customTheta - customThetaNoise, customTheta + customThetaNoise);
  for (auto &p : particles) {
    Point noise = Point::mkRandomPolar(customNoise);
    p.first = FieldPosition(customX + noise.x, customY + noise.y,
                            dirDistribution(generator));
  }
  logger.log("Applying a customReset at x: %f, y: %f, theta: %f, with noise: "
             "%f, noiseTheta: %f",
             customX, customY, customTheta, customNoise, customThetaNoise);
}

void FieldPF::draw(cv::Mat &img, Utils::CameraState *cs) const {
  if (cs == nullptr) {
    std::string msg("FieldPF::draw: cameraState is null");
    logger.error("%s", msg.c_str());
    throw std::logic_error(msg);
  }
  if (cs->_model == nullptr) {
    std::string msg("FieldPF::draw: cameraState is null");
    logger.error("%s", msg.c_str());
    throw std::logic_error(msg);
  }
  Field::Field::drawField(img);
  for (unsigned int i = 0; i < nbParticles(); i++) {
    getParticle(i).tag(img, 0.0, cv::Scalar(getParticleQ(i) * 255, 0, 0));
  }
  representativeParticle.tag(img, 0.0, cv::Scalar(0, 0, 255), 3);
// WARNING: All of this is not so useful now that TopView is updated every n seconds
//          Was also producing errors when robot was falling backward
//  // Extremum points in the image referential
//  std::vector<Point> imgCorners;
//  imgCorners.push_back(Point(-1, -1));
//  imgCorners.push_back(Point(1, -1));
//  imgCorners.push_back(Point(1, 1));
//  imgCorners.push_back(Point(-1, 1));
//  imgCorners.push_back(Point(-1, -1));
//  // Extremum points in the robot referential
//  std::vector<Point> imgCornersInRobot;
//  for (const Point &p : imgCorners) {
//    Eigen::Vector2d pixel(p.x, p.y);
//    Eigen::Vector3d viewVector, pos;
//    viewVector = cs->_model->cameraPixelToViewVector(cs->_params, pixel);
//    bool success = cs->_model->cameraViewVectorToWorld(viewVector, pos);
//    if (!success) {
//      std::cerr << "FieldPF::draw: failed cameraViewVectorToWorld: " << viewVector << std::endl;
//      continue;
//    }
//    pos = 100 * cs->_model->frameInSelf("origin", pos); // m to cm
//    imgCornersInRobot.push_back(Point(pos[0], pos[1]));
//  }
//  // Getting points in the field referential
//  Point robotPos = Point(representativeParticle.getRobotPositionCV());
//  Angle robotDir = representativeParticle.getOrientation();
//  std::vector<Point> imgCornersInField;
//  for (const Point &p : imgCornersInRobot) {
//    imgCornersInField.push_back(robotPos + p.rotation(robotDir));
//  }
//  for (size_t srcPoint = 0; srcPoint < imgCornersInField.size() - 1;
//       srcPoint++) {
//    cv::Point srcField = imgCornersInField[srcPoint].toCV();
//    cv::Point dstField = imgCornersInField[srcPoint + 1].toCV();
//    cv::Point srcImg = Field::Field::fieldToImg(img, srcField);
//    cv::Point dstImg = Field::Field::fieldToImg(img, dstField);
//    double vecThickness = 2 * Field::Field::getScale(img);
//    cv::line(img, srcImg, dstImg, cv::Scalar(128, 0, 0), vecThickness);
//  }
}

void FieldPF::updateRepresentativeQuality() {
  // TODO might be improved
  int nbGoodParticles = 0;
  const Point &repPos = representativeParticle.getRobotPosition();
  const Angle &repDir = representativeParticle.getOrientation();
  for (auto &p : particles) {
    const Point &pos = p.first.getRobotPosition();
    const Angle &dir = p.first.getOrientation();
    double dist = pos.getDist(repPos);
    double diffAngle = std::fabs((dir - repDir).getSignedValue());

    // TODO: The particle quality?
    // p.second=0.5/(diffAngle+1.0)+0.5/(dist+1.0);
    if (dist < tolDist && diffAngle < tolDiffAngle) {
      nbGoodParticles += 1;
    }
  }
  representativeQuality = nbGoodParticles / (double)particles.size();
}

void FieldPF::updateRepresentativeParticle() {

  int N = particles.size();
  Eigen::VectorXd M = particles[0].first.toVector();
  // Better way of calculating avg angle exists but this one should be
  // good enough
  double x(0), y(0); // computing avg angle
  for (int i = 1; i < N; i++) {
    M = M + particles[i].first.toVector();
    x += cos(particles[i].first.getOrientation());
    y += sin(particles[i].first.getOrientation());
  }
  M = (1.0 / (double)N) * M;
  M(2) = rad2deg(atan2(y, x));
  representativeParticle.setFromVector(M);
}

void FieldPF::updateInternalValues() {
  ParticleFilter::updateInternalValues();
  updateToGoalQuality();
}

Angle FieldPF::getAngleToGoal(const GoalKey &gk) const {
  try {
    return goalDirections.at(gk);
  } catch (const std::out_of_range &exc) {
    ostringstream oss;
    oss << "Error while getting Angle to goal, goalKey not found: "
        << "goal: " << gk.first << ", post: " << gk.second << std::endl;
    throw std::runtime_error(oss.str());
  }
}

double FieldPF::angleToGoalQuality(const QualityKey &qk) const {
  try {
    return goalQualities.at(qk);
  } catch (const std::out_of_range &exc) {
    ostringstream oss;
    oss << "Error while getting Angle to goal Quality, qualityKey not found: "
        << "goal: " << std::get<0>(qk) << ", post: " << std::get<1>(qk)
        << ", errTol: " << std::get<2>(qk) << std::endl;
    throw std::runtime_error(oss.str());
  }
}

Angle FieldPF::getAngleToGoal() { return getAngleToGoal(GoalKey(1, 0)); }

double FieldPF::angleToGoalQuality() {
  return angleToGoalQuality(QualityKey(1, 0, 8));
}

double FieldPF::getQuality() { return representativeQuality; }

cv::Point2d FieldPF::getRobotPosition() {
  return cv::Point2d(representativeParticle.x() * 0.01,
                     representativeParticle.y() * 0.01);
}

cv::Point2d FieldPF::getCenterPosition() {
  return cv::Point2d(-representativeParticle.x() * 0.01,
                     -representativeParticle.y() * 0.01);
}

cv::Point2d FieldPF::getLeftGoalPosition() {
  cv::Point2d g(Field::Field::getAdvGoal(1).x * 0.01,
                Field::Field::getAdvGoal(1).y * 0.01); // this is cm
  return g - getRobotPosition();
}

cv::Point2d FieldPF::getRightGoalPosition() {
  cv::Point2d g(Field::Field::getAdvGoal(-1).x * 0.01,
                Field::Field::getAdvGoal(-1).y * 0.01); // this is cm
  return g - getRobotPosition();
}

cv::Point2d FieldPF::getCenterPositionInSelf() {
  cv::Point2d p = getCenterPosition();
  Angle o = representativeParticle.getOrientation();
  cv::Point2d res;
  res.x = p.x * cos(o) + p.y * sin(o);
  res.y = -p.x * sin(o) + p.y * cos(o);
  return res;
}

cv::Point2d FieldPF::getLeftGoalPositionInSelf() {
  cv::Point2d p = getLeftGoalPosition();
  Angle o = representativeParticle.getOrientation();
  cv::Point2d res;
  res.x = p.x * cos(o) + p.y * sin(o);
  res.y = -p.x * sin(o) + p.y * cos(o);

  return res;
}
cv::Point2d FieldPF::getRightGoalPositionInSelf() {
  cv::Point2d p = getRightGoalPosition();
  Angle o = representativeParticle.getOrientation();
  cv::Point2d res;
  res.x = p.x * cos(o) + p.y * sin(o);
  res.y = -p.x * sin(o) + p.y * cos(o);

  return res;
}

Angle FieldPF::getOrientation() {
  return representativeParticle.getOrientation();
}

bool FieldPF::isResetPending() const {
  /// No need for thread safety since it is not setting variables
  return resetType != ResetType::None;
}

void FieldPF::importFromRhIO() {
  resamplingRatio = rhioNode->getValueFloat("resamplingRatio").value;
  tolDist = rhioNode->getValueFloat("tolDist").value;
  tolDiffAngle = rhioNode->getValueFloat("tolDiffAngle").value;
  fallNoise = rhioNode->getValueFloat("fallNoise").value;
  fallNoiseTheta = rhioNode->getValueFloat("fallNoiseTheta").value;
  borderNoise = rhioNode->getValueFloat("borderNoise").value;
  borderNoiseTheta = rhioNode->getValueFloat("borderNoiseTheta").value;
  customX = rhioNode->getValueFloat("customX").value;
  customY = rhioNode->getValueFloat("customY").value;
  customNoise = rhioNode->getValueFloat("customNoise").value;
  customTheta = rhioNode->getValueFloat("customTheta").value;
  customThetaNoise = rhioNode->getValueFloat("customThetaNoise").value;
}

void FieldPF::publishToRhIO() {
  rhioNode->setFloat("goalDir", getAngleToGoal().getSignedValue());
  rhioNode->setFloat("goalDirQ", angleToGoalQuality());
  rhioNode->setFloat("leftGoalDir",
                     getAngleToGoal(GoalKey(1, 1)).getSignedValue());
  rhioNode->setFloat("rightGoalDir",
                     getAngleToGoal(GoalKey(1, -1)).getSignedValue());
  rhioNode->setFloat("posX", representativeParticle.x());
  rhioNode->setFloat("posY", representativeParticle.y());
  rhioNode->setFloat("azimuth",
                     representativeParticle.getOrientation().getSignedValue());
  rhioNode->setFloat("posQ", representativeQuality);
}

std::string FieldPF::getName(ResetType t) {
  try {
    return resetNames.at(t);
  } catch (const std::out_of_range & exc) {
    logger.log("Failed to get name for type: %d", t);
    throw;
  }
}

}
}
