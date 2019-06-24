#include <iostream>

#include "FieldPF.hpp"

#include "rhoban_utils/logging/logger.h"

#include <hl_monitoring/top_view_drawer.h>
#include <robocup_referee/constants.h>
#include <hl_communication/position.pb.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
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
using namespace hl_monitoring;
using namespace hl_communication;

namespace Vision
{
namespace Localisation
{
std::map<FieldPF::ResetType, std::string> FieldPF::resetNames = { { ResetType::None, "None" },
                                                                  { ResetType::Uniform, "Uniform" },
                                                                  { ResetType::BordersRight, "BordersRight" },
                                                                  { ResetType::BordersLeft, "BordersLeft" },
                                                                  { ResetType::Borders, "Borders" },
                                                                  { ResetType::Fall, "Fall" },
                                                                  { ResetType::Custom, "Custom" } };

FieldPF::FieldPF()
  : ParticleFilter(), resetType(ResetType::None), errorTols({ 8 }), resamplingRatio(0.0), tolDist(1), tolDiffAngle(15)
{
  RhIO::Root.newChild("/localisation/field/fieldPF");
  rhioNode = &(RhIO::Root.child("/localisation/field/fieldPF"));
  rhioNode->newFloat("resamplingRatio")
      ->defaultValue(0)
      ->minimum(0)
      ->maximum(1)
      ->comment("Ratio of particles resampled");
  rhioNode->newFloat("posX")->defaultValue(0);
  rhioNode->newFloat("posY")->defaultValue(0);
  rhioNode->newFloat("azimuth")->defaultValue(0);
  rhioNode->newFloat("posQ")->defaultValue(0)->minimum(0)->maximum(1);
  rhioNode->newFloat("tolDist")->defaultValue(tolDist)->minimum(0)->maximum(6);
  rhioNode->newFloat("tolDiffAngle")->defaultValue(tolDiffAngle)->minimum(0)->maximum(180);
  rhioNode->newFloat("fallNoise")
      ->defaultValue(0.25)
      ->minimum(0)
      ->maximum(2)
      ->comment("Maximal noise applied when falling [m]");
  rhioNode->newFloat("fallNoiseTheta")
      ->defaultValue(20)
      ->minimum(0)
      ->maximum(180)
      ->comment("When falling, uniform noise in [-x,x] is applied [deg]");
  rhioNode->newFloat("borderNoise")
      ->defaultValue(0.5)
      ->minimum(0)
      ->maximum(2)
      ->comment("Maximal noise applied on 'x' coordinate on borderReset [m]");
  rhioNode->newFloat("borderNoiseTheta")
      ->defaultValue(10)
      ->minimum(0)
      ->maximum(180)
      ->comment("Orientation noise applied at border reset in [-x,x] [deg]");
  rhioNode->newFloat("borderExtraDist")
      ->defaultValue(0.1)
      ->minimum(0)
      ->maximum(1.0)
      ->comment("Distance to the center line when the robot enters from  the side [m]");
  rhioNode->newFloat("customX")
      ->defaultValue(0)
      ->minimum(-Constants::field.field_length / 2)
      ->maximum(Constants::field.field_length / 2)
      ->comment("X-Position used for customReset [m]");
  rhioNode->newFloat("customY")
      ->defaultValue(0)
      ->minimum(-Constants::field.field_width / 2)
      ->maximum(Constants::field.field_width / 2)
      ->comment("Y-position used for customReset [m]");
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
      ->defaultValue(0.5)
      ->minimum(0)
      ->maximum(4)
      ->comment("Uniform noise used for custom reset [m]");
  rhioNode->newFloat("fieldQDecrease")
      ->defaultValue(-10)
      ->minimum(-1)
      ->maximum(-20)
      ->comment("Changing speed of decreasing fieldQ");
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
  if (previousType != ResetType::None)
  {
    logger.warning("Overwriting a reset of type: '%s' with a reset of type '%s'", old_name.c_str(), new_name.c_str());
  }
}

void FieldPF::cancelPendingReset(ResetType t)
{
  ResetType previousType;
  resetMutex.lock();
  previousType = resetType;
  if (previousType == t)
  {
    resetType = ResetType::None;
  }
  resetMutex.unlock();
  if (previousType == t)
  {
    std::string previous_name = resetNames.at(t);
    logger.log("A reset of type '%s' has been canceled", previous_name.c_str());
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

void FieldPF::applyPendingReset()
{
  ResetType pendingReset;
  resetMutex.lock();
  pendingReset = resetType;
  resetType = ResetType::None;
  resetMutex.unlock();

  applyReset(pendingReset);
}

void FieldPF::applyReset(ResetType t)
{
  if (t == ResetType::None)
    return;

  std::string reset_name = resetNames.at(t);
  logger.log("Applying a reset of type: '%s'", reset_name.c_str());

  double mins[3] = { -Constants::field.field_length / 2, -Constants::field.field_width / 2, 0 };
  double maxs[3] = { Constants::field.field_length / 2, Constants::field.field_width / 2, 360 };

  switch (t)
  {
    case Uniform:
    {
      ParticleFilter::initializeAtUniformRandom(mins, maxs, particles.size());
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
    case Fall:
    {
      fallReset();
      break;
    }
    case Custom:
    {
      customReset();
      break;
    }
    default:
    {
    }
  }
  updateInternalValues();
}

void FieldPF::step(Controller<FieldPosition>& ctrl, const std::vector<Observation<FieldPosition>*>& observations,
                   double elapsedTime)
{
  ResetType stepReset;
  resetMutex.lock();
  stepReset = resetType;
  resetType = ResetType::None;
  resetMutex.unlock();

  // If we are doing a reset, make sure we are up to date with rhio values
  if (stepReset != None)
  {
    importFromRhIO();
  }

  if (stepReset == ResetType::None)
  {
    double mins[3] = { -Constants::field.field_length / 2, -Constants::field.field_width / 2, 0 };
    double maxs[3] = { Constants::field.field_length / 2, Constants::field.field_width / 2, 360 };
    // If no reset is planned, apply observations, resampling if required and return
    ParticleFilter::step(ctrl, observations, elapsedTime);
    partialUniformResampling(resamplingRatio, mins, maxs);
    return;
  }
  else
  {
    applyReset(stepReset);
  }

  // After a reset, we apply odometry but not observations
  // (robot might have moved between request of reset and now)
  ParticleFilter::step(ctrl, elapsedTime);

  // After a reset, update internal values
  updateInternalValues();
}

void FieldPF::resetOnLines(int side)
{
  auto generator = rhoban_random::getRandomEngine();
  // According to rules, robot start in its own half
  double xOffset = Constants::field.penalty_mark_dist - Constants::field.field_length / 2;
  std::uniform_real_distribution<double> xDistribution(-borderNoise, borderNoise);
  std::uniform_real_distribution<double> dirNoiseDistribution(-borderNoiseTheta, borderNoiseTheta);
  std::uniform_int_distribution<int> sideDistribution(0, 1);

  for (auto& p : particles)
  {
    double x = xOffset + xDistribution(generator);
    int currSide;
    // 0 * 2 - 1 = -1 and 1 * 2 - 1 = 1
    if (side > 0)
    {
      currSide = 1;
    }
    else if (side < 0)
    {
      currSide = -1;
    }
    // Side is unknown: sampling
    else
    {
      currSide = sideDistribution(engine) == 0 ? 1 : -1;
    }
    double y = currSide * (Constants::field.field_width / 2 + borderExtraDist);
    double dirNoise = dirNoiseDistribution(generator);
    double dir = -currSide * 90;
    p.first = FieldPosition(x, y, Angle(dir + dirNoise).getSignedValue());
  }
  logger.log("Reset particles on borders");
}

void FieldPF::fallReset()
{
  auto generator = rhoban_random::getRandomEngine();
  std::uniform_real_distribution<double> dirDistribution(-fallNoiseTheta, fallNoiseTheta);
  for (auto& p : particles)
  {
    Point move = Point::mkRandomPolar(fallNoise);
    p.first.move(move);
    p.first.rotate(dirDistribution(generator));
  }
  logger.log("Add noise on particles due to a fall");
}

void FieldPF::customReset()
{
  auto generator = rhoban_random::getRandomEngine();
  std::uniform_real_distribution<double> dirDistribution(customTheta - customThetaNoise,
                                                         customTheta + customThetaNoise);
  for (auto& p : particles)
  {
    Point noise = Point::mkRandomPolar(customNoise);
    p.first = FieldPosition(customX + noise.x, customY + noise.y, dirDistribution(generator));
  }
  logger.log("Applying a customReset at x: %f, y: %f, theta: %f, with noise: "
             "%f, noiseTheta: %f",
             customX, customY, customTheta, customNoise, customThetaNoise);
}

static void tag(cv::Mat& img, const FieldPosition& p, const TopViewDrawer& drawer,
                const cv::Scalar& color = cv::Scalar(255, 0, 255), int thickness = 1)
{
  cv::Point2f pos_field = p.getRobotPositionCV();
  Angle orientation = p.getOrientation();
  double vec_length = 0.2;  //[m]
  cv::Point2f delta(cos(orientation) * vec_length, sin(orientation) * vec_length);
  double radius = 3;
  cv::Point2f end_field = pos_field + delta;

  cv::Point pos_img = drawer.getImgFromField(Constants::field, pos_field);
  cv::Point end_img = drawer.getImgFromField(Constants::field, end_field);

  cv::circle(img, pos_img, radius, color, thickness);
  cv::line(img, pos_img, end_img, color, thickness);
}

void FieldPF::draw(cv::Mat& img) const
{
  TopViewDrawer drawer(cv::Size(img.cols, img.rows));

  img = drawer.getImg(Constants::field);
  for (unsigned int i = 0; i < nbParticles(); i++)
  {
    const FieldPosition& p = getParticle(i);
    tag(img, p, drawer, cv::Scalar(255, 0, 0));
  }
  tag(img, representativeParticle, drawer, cv::Scalar(0, 0, 255), 3);
}

void FieldPF::updateRepresentativeQuality()
{
  hl_communication::WeightedPose weighted_pose = vectorEM[0];
  const PoseDistribution& pose = weighted_pose.pose();
  const PositionDistribution& pos = pose.position();

  float data[4] = { pos.uncertainty(0), pos.uncertainty(1), pos.uncertainty(1), pos.uncertainty(2) };
  cv::Mat covMat = cv::Mat(2, 2, CV_32F, data);

  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(covMat, eigenvalues, eigenvectors);

  fieldQDecrease = rhioNode->getValueFloat("fieldQDecrease").value;
  representativeQuality =
      weighted_pose.probability() * exp(fieldQDecrease * eigenvalues.at<float>(0) * eigenvalues.at<float>(1));
}
/*
cv::Mat FieldPF::positionsFromParticles()
{
  cv::Mat samples;
  Eigen::VectorXd M;
  cv::Mat tmp;
  int nbParticles = particles.size();

  for (int i = 0; i < nbParticles; i++)
  {
    M = particles[i].first.toVector();
    tmp = cv::Mat(1, 2, CV_64F, (void*)&M[0]).clone();
    samples.push_back(tmp);
  }
  return samples;
}

cv::Mat FieldPF::anglesFromParticles()
{
  cv::Mat M;
  int nbParticles = particles.size();
  cv::Mat tmp;

  for (int i = 0; i < nbParticles; i++)
  {
    M.push_back(particles[i].first.getOrientation());
  }
  }*/

void FieldPF::exportParticles(cv::Mat* pos, cv::Mat* angle)
{
  int nbParticles = particles.size();
  Eigen::VectorXd M;
  cv::Mat tmp;

  *pos = cv::Mat(nbParticles, 2, CV_64F);
  *angle = cv::Mat(nbParticles, 1, CV_64F);

  for (int i = 0; i < nbParticles; i++)
  {
    M = particles[i].first.toVector();
    pos->at<double>(i, 0) = M[0];
    pos->at<double>(i, 1) = M[1];
    angle->at<double>(i, 0) = M[2];
  }
}

void FieldPF::updateRepresentativeParticle()
{
  cv::Mat pos, angle;
  exportParticles(&pos, &angle);

  vectorEM.clear();

  int max_clusters = 5;

  vectorEM = updateEM(pos, angle, max_clusters);

  hl_communication::WeightedPose weighted_pose = vectorEM[0];

  const PoseDistribution& pose = weighted_pose.pose();
  Eigen::VectorXd result(3);

  result << pose.position().x(), pose.position().y(), rad2deg(pose.dir().mean());

  representativeParticle.setFromVector(result);
}

std::vector<hl_communication::WeightedPose> FieldPF::getPositionsFromClusters()
{
  return vectorEM;
}

void FieldPF::updateInternalValues()
{
  ParticleFilter::updateInternalValues();
}

double FieldPF::getQuality()
{
  return representativeQuality;
}

cv::Point2d FieldPF::getRobotPosition()
{
  return cv::Point2d(representativeParticle.x(), representativeParticle.y());
}

cv::Point2d FieldPF::getCenterPositionInSelf()
{
  cv::Point2d p = -getRobotPosition();
  Angle o = representativeParticle.getOrientation();
  cv::Point2d res;
  res.x = p.x * cos(o) + p.y * sin(o);
  res.y = -p.x * sin(o) + p.y * cos(o);
  return res;
}

Angle FieldPF::getOrientation()
{
  return representativeParticle.getOrientation();
}

bool FieldPF::isResetPending() const
{
  /// No need for thread safety since it is not setting variables
  return resetType != ResetType::None;
}

void FieldPF::importFromRhIO()
{
  resamplingRatio = rhioNode->getValueFloat("resamplingRatio").value;
  tolDist = rhioNode->getValueFloat("tolDist").value;
  tolDiffAngle = rhioNode->getValueFloat("tolDiffAngle").value;
  fallNoise = rhioNode->getValueFloat("fallNoise").value;
  fallNoiseTheta = rhioNode->getValueFloat("fallNoiseTheta").value;
  borderNoise = rhioNode->getValueFloat("borderNoise").value;
  borderNoiseTheta = rhioNode->getValueFloat("borderNoiseTheta").value;
  borderExtraDist = rhioNode->getValueFloat("borderExtraDist").value;
  customX = rhioNode->getValueFloat("customX").value;
  customY = rhioNode->getValueFloat("customY").value;
  customNoise = rhioNode->getValueFloat("customNoise").value;
  customTheta = rhioNode->getValueFloat("customTheta").value;
  customThetaNoise = rhioNode->getValueFloat("customThetaNoise").value;
}

void FieldPF::publishToRhIO()
{
  rhioNode->setFloat("posX", representativeParticle.x());
  rhioNode->setFloat("posY", representativeParticle.y());
  rhioNode->setFloat("azimuth", representativeParticle.getOrientation().getSignedValue());
  rhioNode->setFloat("posQ", representativeQuality);
}

std::string FieldPF::getName(ResetType t)
{
  try
  {
    return resetNames.at(t);
  }
  catch (const std::out_of_range& exc)
  {
    logger.log("Failed to get name for type: %d", t);
    throw;
  }
}

void FieldPF::initializeAtUniformRandom(unsigned int particlesNb)
{
  double mins[3] = { -Constants::field.field_length / 2, -Constants::field.field_width / 2, 0 };
  double maxs[3] = { Constants::field.field_length / 2, Constants::field.field_width / 2, 360 };
  ParticleFilter::initializeAtUniformRandom(mins, maxs, particlesNb);
}

}  // namespace Localisation
}  // namespace Vision
