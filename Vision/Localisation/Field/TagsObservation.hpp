#pragma once

#include "Localisation/Field/SerializableFieldObservation.hpp"
#include <string>

namespace Vision
{
namespace Localisation
{

/**
 * This observation is not working anymore, in order to be updated, it would require to
 * store the position of the tags at an appropriate location (unclear where it should be done).
 * Then, initialization of 'knownTags' in getTagPosInParticleSelf and potential should be updated.
 */
class TagsObservation : public SerializableFieldObservation
{
public:
  // Id of the marker
  int id;

  // Position of the marker observation [m]
  cv::Point3f seenPos;

  // Deviation on the marker observation [m]
  cv::Point3f stdDev;

  /// Robot height at the given time [m]
  double robotHeight;

  // Weight of the observation
  double weight;

  // Required for numerical reasons
  static double pError;
  // Until which angle (in deg) the angleScore is 1
  static double angleFlatTol;
  // After which angle (in deg) the angleScore is pError
  static double angleMaxErr;
  // Until which distance error (in m) the normScore is still 1
  static double normFlatTol;
  // After which distance error (in m) the normScore is pError
  static double normMaxErr;
  // After which position error (in m) the distanceScore reaches 0.5
  static double distFactor;
  /// true -> score = angleScore * normScore
  /// false-> score = distanceScore
  static bool angleMode;

public:
  TagsObservation();

  TagsObservation(const int& id_, const cv::Point3f& seenPos_, const cv::Point3f& seenDev_, double robotHeight_,
                  double weight);

  cv::Point3f getTagPosInParticleSelf(int tagId, const FieldPosition& p) const;

  // Get direction vector from camera to target point in robot basis
  cv::Point3f getSeenVec(const cv::Point3f& posInSelf) const;

  virtual double potential(const FieldPosition& p) const override;

  static void bindWithRhIO();
  static void importFromRhIO();

  std::string getClassName() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name);
  Json::Value toJson() const;

  double getMinScore() const override;

  virtual std::string toStr() const override;
};
}  // namespace Localisation
}  // namespace Vision
