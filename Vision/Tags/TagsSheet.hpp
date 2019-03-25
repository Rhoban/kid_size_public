#pragma once

#include "TagsCollection.hpp"

/// Represent a 3*2 sheet of aruco markers with a rectangular pattern
///           ----dx--->
///    |    id[0]     id[1]
///    |
///   dy
///    |
///   \ /   id[2]     id[3]
///
///
///
///         id[4]     id[5]
class TagsSheet : public TagsCollection
{
public:
  TagsSheet();
  TagsSheet(double marker_size, const Eigen::Vector3d& dx, const Eigen::Vector3d& dy,
            const Eigen::Vector3d& sheet_center, const std::vector<int>& markers_ids);

  virtual std::map<int, ArucoTag> getMarkers() const override;

  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_path) override;
  virtual std::string getClassName() const override;

private:
  /// Size of an aruco marker (all markers of a sheet have the same size) [m]
  double marker_size;

  /// Spacing between two columns of tags [m]
  Eigen::Vector3d dx;

  /// Delta between two lines of tags (from [m]
  Eigen::Vector3d dy;

  /// Position of the center of the sheet (middle of id[2] and id[3]) [m]
  Eigen::Vector3d sheet_center;

  /// The list of the marker ids
  std::vector<int> markers_ids;
};
