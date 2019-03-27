#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_utils/angle.h"
#include "rhoban_geometry/point.h"
#include "rhoban_unsorted/particle_filter/particle.h"

namespace Vision
{
namespace Localisation
{
/// values[0] -> position X [m]
/// values[1] -> position Y [m]
/// values[2] -> orientation (in degrees, corresponding to the description in
///                           class Angle)
///              top   ->  0
///              right -> 90
class FieldPosition : public rhoban_unsorted::Particle<3>
{
public:
  FieldPosition() : Particle()
  {
  }
  FieldPosition(double xInit, double yInit, double angleInit);
  FieldPosition(const FieldPosition& other) : Particle(other)
  {
  }

  virtual ~FieldPosition();

  double x() const
  {
    return values[0];
  }
  double y() const
  {
    return values[1];
  }

  rhoban_utils::Angle getOrientation() const;
  rhoban_geometry::Point getRobotPosition() const;
  cv::Point2f getRobotPositionCV() const;

  /// Return the position of 'pos_in_field' in robot referential
  rhoban_geometry::Point getFieldPosInSelf(const rhoban_geometry::Point& pos_in_field) const;

  void move(rhoban_geometry::Point& dist);  // dist is given in the robot referential
  void rotate(const rhoban_utils::Angle& rotation);

  rhoban_unsorted::Particle<3>* clone() const
  {
    return new FieldPosition(*this);
  }
};
}  // namespace Localisation
}  // namespace Vision
