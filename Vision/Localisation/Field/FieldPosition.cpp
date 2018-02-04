#include "FieldPosition.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "Field/Field.hpp"

using namespace Vision::Utils;
using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace Vision {
namespace Localisation {

FieldPosition::FieldPosition(double xInit, double yInit, double angle) {
  values[0] = xInit;
  values[1] = yInit;
  values[2] = angle;
}

FieldPosition::~FieldPosition() {}

Angle FieldPosition::getOrientation() const { return Angle(values[2]); }

Point FieldPosition::getRobotPosition() const {
  return Point(values[0], values[1]);
}

cv::Point FieldPosition::getRobotPositionCV() const {
  return cv::Point((int)values[0], (int)values[1]);
}

Point FieldPosition::getFieldPosInSelf(const Point & pos_in_field) const {
  Point offset = pos_in_field - Point(x(), y());
  return offset.rotation(-getOrientation());
}


// dist is given in the robot referential
void FieldPosition::move(Point &dist) {
  Point original = getRobotPosition();
  Point newPos = original + dist.rotation(getOrientation());
  values[0] = newPos.x;
  values[1] = newPos.y;
}

void FieldPosition::rotate(const Angle &rotation) {
  values[2] = (getOrientation() + rotation).getSignedValue();
}

void FieldPosition::tag(cv::Mat &img, Angle pan, const cv::Scalar &color,
                        int thickness) const {
  double scaledThickness = thickness * Field::Field::getScale(img) * 2;
  double vecLength = scaledThickness * 10;
  cv::Point src = getRobotPositionCV();
  Angle orientation = getOrientation() + pan;
  double dx = cos(orientation);
  double dy = sin(orientation);
  cv::Point delta = cv::Point(dx * vecLength, dy * vecLength);
  double radius = 3 * scaledThickness;
  cv::Point srcImg = Field::Field::fieldToImg(img, src);
  cv::Point endImg = Field::Field::fieldToImg(img, src + delta);

  // circle(img, srcImg, radius, color, CV_FILLED);
  circle(img, srcImg, radius, color, scaledThickness);
  line(img, srcImg, endImg, color, scaledThickness);
}

}
}
