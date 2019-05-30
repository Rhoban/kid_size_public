#pragma once

#include <Localisation/Field/SerializableFieldObservation.hpp>
#include <CameraState/CameraState.hpp>

#include <hl_monitoring/field.h>

namespace Vision
{
namespace Localisation
{
class FeatureObservation : public SerializableFieldObservation
{
public:
  /**
   * The type of feature observed
   */
  hl_monitoring::Field::POIType poiType;

  /**
   * Direction of the feature
   */
  rhoban_geometry::PanTilt panTilt;

  /// [m]
  double robotHeight;

  /// Several feature observations can be merged into a single one,
  /// In this case, the weight of the observation is increased because
  /// observations are consistent
  double weight;

  /// Minimal score for an observation of weight 1
  static double pError;

  /// [deg]
  static double maxAngleError;
  static double tolAngleError;

  /// Limit used to determine if observations are similars
  static double similarAngleLimit;

  /// [m]
  static double maxCartError;
  static double tolCartError;

  /// Limit used to determine if observations are similars
  static double similarPosLimit;

  /// Used to weight more observations which represent large clusters
  static double weightRatio;

public:
  FeatureObservation();

  FeatureObservation(hl_monitoring::Field::POIType poiType, const rhoban_geometry::PanTilt& panTiltToFeature,
                     double robotHeight, double weight = 1);

  /**
   * Return true on success, false if it fails to get seen direction (observation is above horizon)
   */
  bool getSeenDir(cv::Point3f* out) const;

  virtual double potential(const FieldPosition& p) const override;

  double potential(const FieldPosition& p, bool debug) const;

  /// Merge 'other' observation into current one:
  /// average angles and cumulate weights
  void merge(const FeatureObservation& other);

  static bool isSimilar(const FeatureObservation& o1, const FeatureObservation& o2);

  static void bindWithRhIO();
  static void importFromRhIO();

  std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

  double getMinScore() const override;

  double getWeightedScore(double score) const;

  std::string getPOITypeName() const;

  virtual std::string toStr() const override;
};
}  // namespace Localisation
}  // namespace Vision
