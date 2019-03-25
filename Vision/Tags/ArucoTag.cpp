#include "ArucoTag.hpp"

ArucoTag::ArucoTag() : marker_id(-1)
{
}

ArucoTag::ArucoTag(int marker_id, double marker_size, const Eigen::Vector3d& marker_center,
                   const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis)
  : marker_id(marker_id), marker_size(marker_size), marker_center(marker_center), x_axis(x_axis), y_axis(y_axis)
{
}
