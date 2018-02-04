#pragma once

#include "Localisation/Field/SerializableFieldObservation.hpp"
#include <string>

namespace Vision {
namespace Localisation {

class CompassObservation : public SerializableFieldObservation {
private:
  typedef rhoban_utils::Angle Angle;
  
  /// Visual compass value which should be provided when facing opponent goal
  static Angle offset;

  /// Angle of the observation by in field referential
  Angle obs;

  static double pError, maxError, sigmoidOffset, sigmoidLambda;

public:
  /// Return the field orientation corresponding to the visual compass orientation
  /// given as a parameter
  static Angle compassToField(Angle visualCompassAngle);

  CompassObservation();

  /// Angle is direction of
  CompassObservation(Angle visualCompassAngle);

  virtual double potential(const FieldPosition & p) const;

  /**
   * Define the compass value when aiming toward center of adversary goal
   * from center of our goal. This should change at half-time
   */
  static void setReference(Angle dirToOppGoal);

  /**
   * Get the compass offset
   */
  static Angle getOffset();
  static void setOffset(Angle offset);

  static void bindWithRhIO();
  static void importFromRhIO();
  static void exportToRhIO();

  std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;

  double getMinScore() const;

  virtual std::string toStr() const override;
};
}
}
