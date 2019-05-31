#include "FeatureObservation.hpp"

#include "CameraState/CameraState.hpp"

#include "RhIO.hpp"

#include "rhoban_utils/util.h"
#include "rhoban_utils/logging/logger.h"

#include "Utils/Interface.h"

#include <robocup_referee/constants.h>

static rhoban_utils::Logger out("FeatureObservation");

using Vision::Utils::CameraState;
using namespace hl_monitoring;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

namespace Vision
{
namespace Localisation
{
double FeatureObservation::pError = 0.2;
double FeatureObservation::weightRatio = 0.2;
// Angle tol
double FeatureObservation::maxAngleError = 10;
double FeatureObservation::tolAngleError = 2;
double FeatureObservation::similarAngleLimit = 2;
// Cart tol
double FeatureObservation::maxCartError = 1.0;
double FeatureObservation::tolCartError = 0.2;
double FeatureObservation::similarPosLimit = 0.5;

static double getScore(double error, double maxError, double tol)
{
  if (error >= maxError)
    return 0;
  if (error <= tol)
    return 1;
  return 1 - (error - tol) / (maxError - tol);
}

FeatureObservation::FeatureObservation()
  : FeatureObservation(Field::POIType::Unknown, PanTilt(Angle(0), Angle(0)), 0, 1)
{
}

FeatureObservation::FeatureObservation(hl_monitoring::Field::POIType poiType_, const PanTilt& panTiltToFeature,
                                       double robotHeight_, double weight_)
  : poiType(poiType_), panTilt(panTiltToFeature), robotHeight(robotHeight_), weight(weight_)
{
}

bool FeatureObservation::getSeenDir(cv::Point3f* out) const
{
  Eigen::Vector3d dir = panTilt.toViewVector();
  if (dir.z() >= 0)
  {
    return false;
  }
  double scale = robotHeight / std::fabs(dir.z());
  *out = eigen2CV(Eigen::Vector3d(scale * dir));
  return true;
}

double FeatureObservation::potential(const FieldPosition& p) const
{
  return potential(p, false);
}

double FeatureObservation::potential(const FieldPosition& p, bool debug) const
{
  double bestScore = 0;
  cv::Point3f seenDir;
  if (!getSeenDir(&seenDir))
  {
    return pError;
  }

  std::ostringstream oss;
  if (debug)
  {
    oss << "Debugging potential for particle (" << p.x() << ", " << p.y() << ", " << p.getOrientation().getSignedValue()
        << ") -> seenDir: " << seenDir << std::endl;
  }

  for (const cv::Point3f& cvFeaturePosInField : Constants::field.getPointsOfInterestByType().at(poiType))
  {
    Point featurePosInField(cvFeaturePosInField.x, cvFeaturePosInField.y);
    // Getting feature pos in
    Point featurePosInRobot = (featurePosInField - p.getRobotPosition()).rotation(-p.getOrientation());
    // Rotation of alpha around robotPos
    cv::Point2f expectedPos = rg2cv2f(featurePosInRobot);
    cv::Point3f expectedDir(expectedPos.x, expectedPos.y, -robotHeight);
    // Computing errors
    double dx = expectedPos.x - seenDir.x;
    double dy = expectedPos.y - seenDir.y;
    double cartDiff = sqrt(dx * dx + dy * dy);
    // aDiff is always positive (angleBetween)
    double aDiff = angleBetween(seenDir, expectedDir).getSignedValue();
    // Computing scores
    double aScore = getScore(aDiff, maxAngleError, tolAngleError);
    double cartScore = getScore(cartDiff, maxCartError, tolCartError);
    double score = std::max(aScore, cartScore);

    if (debug)
    {
      oss << "\t: " << featurePosInField << " -> expDir: " << expectedDir << ", "
          << "angle: (" << aDiff << ", " << aScore << ") "
          << "cartesian: (" << cartDiff << ", " << cartScore << ")" << std::endl;
    }

    // If current post has a better score, use it
    if (score > bestScore)
    {
      bestScore = score;
    }
  }

  if (debug)
  {
    out.log("%s", oss.str().c_str());
  }

  return getWeightedScore(bestScore * (1 - pError) + pError);
}

void FeatureObservation::merge(const FeatureObservation& other)
{
  panTilt.pan = Angle::weightedAverage(panTilt.pan, weight, other.panTilt.pan, other.weight);
  panTilt.tilt = Angle::weightedAverage(panTilt.tilt, weight, other.panTilt.tilt, other.weight);
  weight = weight + other.weight;
}

bool FeatureObservation::isSimilar(const FeatureObservation& o1, const FeatureObservation& o2)
{
  if (o1.poiType != o2.poiType)
    return false;
  cv::Point3f seenDir1, seenDir2;
  if (!o1.getSeenDir(&seenDir1) || !o2.getSeenDir(&seenDir2))
  {
    return false;
  }
  Angle aDiff = angleBetween(seenDir1, seenDir2);
  double dx = seenDir1.x - seenDir2.x;
  double dy = seenDir1.y - seenDir2.y;
  double dist = dx * dx + dy * dy;
  bool angleSimilar = aDiff.getValue() < similarAngleLimit;  // aDiff is in [0,180]
  bool posSimilar = dist < similarPosLimit;
  return angleSimilar || posSimilar;
}

void FeatureObservation::bindWithRhIO()
{
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/pError")
      ->defaultValue(pError)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("The false positive probability");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/maxAngleError")
      ->defaultValue(maxAngleError)
      ->minimum(0.0)
      ->maximum(180)
      ->comment("The maximum difference between expectation and observation [deg]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/tolAngleError")
      ->defaultValue(tolAngleError)
      ->minimum(0.0)
      ->maximum(10)
      ->comment("The tolerance between expectation and observation [deg]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/similarAngleLimit")
      ->defaultValue(similarAngleLimit)
      ->minimum(0.0)
      ->maximum(90.0)
      ->comment("Maximal angular difference for similar observations (merge) [deg]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/maxCartError")
      ->defaultValue(maxCartError)
      ->minimum(0.0)
      ->maximum(180)
      ->comment("The maximum difference between expectation and observation [m]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/tolCartError")
      ->defaultValue(tolCartError)
      ->minimum(0.0)
      ->maximum(10)
      ->comment("The tolerance between expectation and observation [m]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/similarCartLimit")
      ->defaultValue(similarPosLimit)
      ->minimum(0.0)
      ->maximum(90.0)
      ->comment("Maximal position difference for similar observations (merge) [m]");
  RhIO::Root.newFloat("/localisation/field/FeatureObservation/weightRatio")
      ->defaultValue(weightRatio)
      ->minimum(0.0)
      ->maximum(90.0)
      ->comment("How is score growing with several particles? pow(score,1+weight*weightRatio)");
}

void FeatureObservation::importFromRhIO()
{
  RhIO::IONode& node = RhIO::Root.child("localisation/field/FeatureObservation");
  pError = node.getValueFloat("pError").value;
  maxAngleError = node.getValueFloat("maxAngleError").value;
  similarAngleLimit = node.getValueFloat("similarAngleLimit").value;
  similarPosLimit = node.getValueFloat("similarCartLimit").value;
}

std::string FeatureObservation::getClassName() const
{
  return "FeatureObservation";
}

Json::Value FeatureObservation::toJson() const
{
  Json::Value v;
  v["poiType"] = getPOITypeName();
  v["robotHeight"] = robotHeight;
  v["pan"] = panTilt.pan.getSignedValue();
  v["tilt"] = panTilt.tilt.getSignedValue();
  return v;
}

void FeatureObservation::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  std::string poiTypeStr;
  rhoban_utils::tryRead(v, "poiType", &poiTypeStr);
  if (poiTypeStr != "")
  {
    poiType = Field::string2POIType(poiTypeStr);
  }
  rhoban_utils::tryRead(v, "robotHeight", &robotHeight);
  double pan = panTilt.pan.getSignedValue();
  double tilt = panTilt.tilt.getSignedValue();
  rhoban_utils::tryRead(v, "pan", &pan);
  rhoban_utils::tryRead(v, "tilt", &tilt);
  panTilt = PanTilt(Angle(pan), Angle(tilt));
}

double FeatureObservation::getMinScore() const
{
  return getWeightedScore(pError);
}

double FeatureObservation::getWeightedScore(double score) const
{
  return pow(score, 1 + (weight - 1) * weightRatio);
}

std::string FeatureObservation::getPOITypeName() const
{
  return Field::poiType2String(poiType);
}

std::string FeatureObservation::toStr() const
{
  std::ostringstream oss;
  oss << "[FeatureObservation: poiType=" << getPOITypeName() << " pan=" << panTilt.pan.getSignedValue()
      << "° tilt=" << panTilt.tilt.getSignedValue() << "°]";
  return oss.str();
}

}  // namespace Localisation
}  // namespace Vision
