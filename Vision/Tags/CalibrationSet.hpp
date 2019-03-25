#pragma once

#include "TagsCollection.hpp"
#include "TagsSheet.hpp"

/// Describe an aruco calibration set designed by Rhoban. It is composed of
/// 10 pannels, each pannel containing two sheets
///
/// P9 is the basis of the robot
/// P0 and P8 are horizontal (flat on the ground)
/// P1 to P7 are vertical
///
///
/// Pk contains the the tags from 12*k to 12*k+11
/// Warning: P9 only contains tags from 108 to 113
///                                                          x
///                                                          |
///                                                          |
///                                                      y----
///
///        |          |          |          |
///        |    P3    |    P4    |    P5    |
///        |          |          |          |
///  ------|---------------------------------------
///        |          |          |          |
///        |          |          |          |
///    P2  |          |          |          |  P6
///        |          |          |          |
///        |          |          |          |
///  ------|    P0    |    P9    |    P8    |------
///        |          |          |          |
///        |          |          |          |
///    P1  |          |          |          |  P7
///        |          |          |          |
///        |          |          |          |
///  ------|----------|----------|--------- |------
class CalibrationSet : public TagsCollection
{
public:
  CalibrationSet();

  virtual std::map<int, ArucoTag> getMarkers() const;

  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_path) override;
  virtual std::string getClassName() const override;

private:
  /// Update the content/position of the sheets
  void updateSheets();

  Eigen::Vector3d getXDir(int panel_idx) const;
  Eigen::Vector3d getYDir(int panel_idx) const;
  Eigen::Vector3d sheet1Center(int panel_idx) const;
  Eigen::Vector3d sheet2Center(int panel_idx) const;
  std::vector<int> getMarkersIndices(int panel_idx, int sheet_idx);

  /// Size of the markers [m]
  double marker_size;

  /// Initial position of the robot in P9 (center of P9 is (0,0)) [m]
  Eigen::Vector2d robot_pos;

  /// Size of a panel (length, width) [m]
  Eigen::Vector2d panel_size;

  /// Thickness of a panel [m]
  double panel_thickness;

  /// Distance between border with connectors to closest sheet center
  double border_to_center_1;

  /// Distance between border without connectors to closest sheet center
  double border_to_center_2;

  /// Horizontal space between two tags [m]
  double sheet_dx;

  /// Vertical space between two tags [m]
  double sheet_dy;

  /// Distance between the center of two sheets on the same panel [m]
  double sheets_spacing;

  /// All the tags sheets of the CalibrationSet
  std::vector<TagsSheet> sheets;
};
