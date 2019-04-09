#include "TagsObservation.hpp"

#include "Utils/Interface.h"

#include "RhIO.hpp"

#include "rhoban_utils/logging/logger.h"

#include <robocup_referee/constants.h>
#include <string>
#include <sstream>
#include <cmath>

static rhoban_utils::Logger out("TagsObservation");

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

namespace Vision
{
namespace Localisation
{
double TagsObservation::pError = std::pow(10, -3);
double TagsObservation::angleFlatTol = 1;
double TagsObservation::angleMaxErr = 5;
double TagsObservation::normFlatTol = 0.05;
double TagsObservation::normMaxErr = 0.5;
double TagsObservation::distFactor = 0.5;
bool TagsObservation::angleMode = true;

TagsObservation::TagsObservation(const int& id_, const cv::Point3f& seenPos_, const cv::Point3f& seenDev_,
                                 double robotHeight_, double weight_)
  : id(id_), seenPos(seenPos_), stdDev(seenDev_), robotHeight(robotHeight_), weight(weight_)
{
}

cv::Point3f TagsObservation::getTagPosInParticleSelf(int tagId, const FieldPosition& p) const
{
  std::map<int, cv::Point3f> knowntags;
  if (knowntags.find(id) == knowntags.end())
  {
    throw std::logic_error("Unknown tag: " + std::to_string(tagId));
  }
  // Getting ground position of the tag in the robot referential
  cv::Point2f ground_pos_in_field;
  ground_pos_in_field.x = knowntags.at(id).x;
  ground_pos_in_field.y = knowntags.at(id).y;
  Point expected_in_self = p.getFieldPosInSelf(cv2rg(ground_pos_in_field));
  // Converting back to cv::Point3f and getting z back
  cv::Point3f pos_in_self;
  pos_in_self.x = expected_in_self.x;
  pos_in_self.y = expected_in_self.y;
  pos_in_self.z = knowntags.at(id).z;

  return pos_in_self;
}

cv::Point3f TagsObservation::getSeenVec(const cv::Point3f& posInSelf) const
{
  return posInSelf - cv::Point3f(0, 0, robotHeight);
}

double TagsObservation::potential(const FieldPosition& p) const
{
  std::map<int, cv::Point3f> knowntags;

  if (knowntags.find(id) == knowntags.end())
  {
    // If a tag is found which is unknown, then the observations is in fault and
    // not the particle
    return 1.0;
  }
  else if (angleMode)
  {
    cv::Point3f pos_in_self = getTagPosInParticleSelf(id, p);
    cv::Point3f seen_dir = getSeenVec(seenPos);
    cv::Point3f expected_dir = getSeenVec(pos_in_self);

    // Compute angle score
    Angle deltaAngle = angleBetween(seen_dir, expected_dir);
    double dAngleDeg = std::fabs(deltaAngle.getSignedValue());
    double angleScore = 1;
    if (dAngleDeg > angleMaxErr)
    {
      angleScore = pError;
    }
    else if (dAngleDeg > angleFlatTol)
    {
      double gamma = (dAngleDeg - angleFlatTol) / (angleMaxErr - angleFlatTol);
      angleScore = (1 - gamma) * (1 - pError) + pError;
    }
    // Compute norm score
    double deltaNorm = std::fabs(cv::norm(seen_dir) - cv::norm(expected_dir));
    double normScore = 1;
    if (deltaNorm > normMaxErr)
    {
      normScore = pError;
    }
    else if (deltaNorm > normFlatTol)
    {
      double gamma = (deltaNorm - normFlatTol) / (normMaxErr - normFlatTol);
      normScore = (1 - gamma) * (1 - pError) + pError;
    }
    return angleScore * normScore;
  }
  else
  {
    cv::Point3f pos_in_self = getTagPosInParticleSelf(id, p);

    double dx = seenPos.x - pos_in_self.x;
    double dy = seenPos.y - pos_in_self.y;
    double dz = seenPos.z - pos_in_self.z;

    double dist = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    double score = pow(1.0 / (1.0 + dist / distFactor), weight);
    return score;
  }
}

void TagsObservation::bindWithRhIO()
{
  RhIO::Root.newFloat("/localisation/field/TagsObservation/angleFlatTol")
      ->defaultValue(angleFlatTol)
      ->minimum(0.0)
      ->maximum(10.0)
      ->comment("[deg] Until which angle error the angleScore is 1");
  RhIO::Root.newFloat("/localisation/field/TagsObservation/angleMaxErr")
      ->defaultValue(angleMaxErr)
      ->minimum(0.0)
      ->maximum(10.0)
      ->comment("[deg] From which angle error the angleScore is pError");
  RhIO::Root.newFloat("/localisation/field/TagsObservation/normFlatTol")
      ->defaultValue(normFlatTol)
      ->minimum(0.0)
      ->maximum(0.5)
      ->comment("[m] Until which norm error the normScore is 1");
  RhIO::Root.newFloat("/localisation/field/TagsObservation/normMaxErr")
      ->defaultValue(normMaxErr)
      ->minimum(0.05)
      ->maximum(1.0)
      ->comment("[m] From which norm error the normScore is pError");
  RhIO::Root.newFloat("/localisation/field/TagsObservation/distFactor")
      ->defaultValue(distFactor)
      ->minimum(0.0)
      ->maximum(10)
      ->comment("[m] The position difference at which disScore is 0.5");
  RhIO::Root.newBool("/localisation/field/TagsObservation/angleMode")
      ->defaultValue(angleMode)
      ->comment("Is angle mode used ? (false -> distMode)");
  RhIO::Root.newFloat("/localisation/field/TagsObservation/pError")
      ->defaultValue(pError)
      ->minimum(std::pow(10, -10))
      ->maximum(1.0)
      ->comment("Minimal potential for an observation with weight 1");
}

void TagsObservation::importFromRhIO()
{
  RhIO::IONode& node = RhIO::Root.child("localisation/field/TagsObservation");
  angleFlatTol = node.getValueFloat("angleFlatTol").value;
  angleMaxErr = node.getValueFloat("angleMaxErr").value;
  normFlatTol = node.getValueFloat("normFlatTol").value;
  normMaxErr = node.getValueFloat("normMaxErr").value;
  distFactor = node.getValueFloat("distFactor").value;
  angleMode = node.getValueBool("angleMode").value;
  pError = node.getValueFloat("pError").value;
}

std::string TagsObservation::getClassName() const
{
  return "TagsObservation";
}

double TagsObservation::getMinScore() const
{
  return pow(pError, weight);
}

std::string TagsObservation::toStr() const
{
  std::ostringstream oss;
  oss << "[TagsObservation: id=" << id << " pos=(" << seenPos.x << ", " << seenPos.y << ", " << seenPos.z << ")]";
  return oss.str();
}

void TagsObservation::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)v;
  (void)dir_name;
  throw std::logic_error("TagsObservation::fromJson: not implemented yet");
}

Json::Value TagsObservation::toJson() const
{
  throw std::logic_error("TagsObservation::fromJson: not implemented yet");
}

}  // namespace Localisation
}  // namespace Vision
