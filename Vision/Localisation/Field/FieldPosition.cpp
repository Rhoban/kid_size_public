#include "FieldPosition.hpp"

#include <opencv2/imgproc/imgproc.hpp>

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace Vision
{
namespace Localisation
{
FieldPosition::FieldPosition(double xInit, double yInit, double angle)
{
  values[0] = xInit;
  values[1] = yInit;
  values[2] = angle;
}

FieldPosition::~FieldPosition()
{
}

Angle FieldPosition::getOrientation() const
{
  return Angle(values[2]);
}

Point FieldPosition::getRobotPosition() const
{
  return Point(values[0], values[1]);
}

cv::Point2f FieldPosition::getRobotPositionCV() const
{
  return cv::Point2f(values[0], values[1]);
}

Point FieldPosition::getFieldPosInSelf(const Point& pos_in_field) const
{
  Point offset = pos_in_field - Point(x(), y());
  return offset.rotation(-getOrientation());
}

// dist is given in the robot referential
void FieldPosition::move(Point& dist)
{
  Point original = getRobotPosition();
  Point newPos = original + dist.rotation(getOrientation());
  values[0] = newPos.x;
  values[1] = newPos.y;
}

void FieldPosition::rotate(const Angle& rotation)
{
  values[2] = (getOrientation() + rotation).getSignedValue();
}

}  // namespace Localisation
}  // namespace Vision
