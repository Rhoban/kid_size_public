#pragma once

#include "Localisation/Field/SerializableFieldObservation.hpp"
#include "Filters/Custom/FieldBorderData.hpp"

namespace Vision {
namespace Localisation {

class ArenaCornerObservation : public SerializableFieldObservation {
private:
  Vision::Filters::FieldBorderData brut_data;
  rhoban_utils::Angle pan;
  rhoban_utils::Angle tilt;
  double weight;

  /// [m]
  double robotHeight;

  std::vector<FieldPosition> candidates;
  std::vector<double> x_candidates;
  std::vector<double> y_candidates;
  int sign_dot;
  double seg_dir;
  static bool debug;
  

public:
  static double pError, maxAngleError, sigmoidOffset, sigmoidLambda;
  static double potential_pos50, potential_angle50;
  static double potential_exp;
  
  ArenaCornerObservation();
  /**
   * panToArenaCorner angle is given in robot referential (left = +, right = -)
   */
  ArenaCornerObservation(const Vision::Filters::FieldBorderData & brut_data_,
                         const rhoban_utils::Angle &panToArenaCorner,
                         const rhoban_utils::Angle &tiltToArenaCorner,
                         double robotHeight_,
                         double weight_ = 1);

  virtual double potential(const FieldPosition &p) const;
  double corner_potential(const FieldPosition &p) const;
  double potential(const FieldPosition & candidate,
		   const FieldPosition &p) const;
  double segment_potential(const FieldPosition &p) const; 
  
  rhoban_utils::Angle getPan() const;
  rhoban_utils::Angle getTilt() const;
  double getWeight() const;
  
  static void bindWithRhIO();
  static void importFromRhIO();

  std::string getClassName() const override;
  Json::Value toJson() const override;
  virtual std::string toStr() const override; 
  void fromJson(const Json::Value & v, const std::string & dir_name) override;

  double getMinScore() const override;
  Vision::Filters::FieldBorderData getBrutData();

  double basic_pot(double x, double x_half) const;
};
}
}
