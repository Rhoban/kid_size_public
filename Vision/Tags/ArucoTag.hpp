#pragma once

#include <Eigen/Core>

class ArucoTag
{
public:
  ArucoTag();

  ArucoTag(int marker_id, double marker_size, const Eigen::Vector3d& marker_center, const Eigen::Vector3d& x_axis,
           const Eigen::Vector3d& y_axis);

  /// The aruco identifier
  int marker_id;

  /// Size of the marker [m]
  double marker_size;

  /// Center of the marker [m]
  Eigen::Vector3d marker_center;

  /// X-Axis of the marker
  Eigen::Vector3d x_axis;

  /// Y-axis of the marker
  Eigen::Vector3d y_axis;
};
