#include "TagsSheet.hpp"

#include <iostream>

TagsSheet::TagsSheet()
  : marker_size(0.09), dx(0.12, 0, 0), dy(0, 0.10, 0), sheet_center(0, 0, 0), markers_ids({ 0, 1, 2, 3, 4, 5 })
{
}
TagsSheet::TagsSheet(double marker_size, const Eigen::Vector3d& dx, const Eigen::Vector3d& dy,
                     const Eigen::Vector3d& sheet_center, const std::vector<int>& markers_ids)
  : marker_size(marker_size), dx(dx), dy(dy), sheet_center(sheet_center), markers_ids(markers_ids)
{
}

std::map<int, ArucoTag> TagsSheet::getMarkers() const
{
  std::map<int, ArucoTag> markers;
  for (int idx = 0; idx < 6; idx++)
  {
    int marker_id = markers_ids[idx];
    double coeffX = (idx % 2 == 0) ? -0.5 : 0.5;
    double coeffY = std::floor(idx / 2) - 1;
    Eigen::Vector3d marker_center = sheet_center + coeffX * dx + coeffY * dy;
    markers[marker_id] = ArucoTag(marker_id, marker_size, marker_center, dx.normalized(), dy.normalized());
  }
  return markers;
}

Json::Value TagsSheet::toJson() const
{
  Json::Value v;
  v["marker_size"] = marker_size;
  v["markers_ids"] = rhoban_utils::vector2Json(markers_ids);
  v["dx"] = rhoban_utils::vector2Json(dx);
  v["dy"] = rhoban_utils::vector2Json(dy);
  v["sheet_center"] = rhoban_utils::vector2Json(sheet_center);
  return v;
}

void TagsSheet::fromJson(const Json::Value& v, const std::string& dir_path)
{
  (void)dir_path;
  rhoban_utils::tryRead(v, "marker_size", &marker_size);
  rhoban_utils::tryReadEigen(v, "dx", &dx);
  rhoban_utils::tryReadEigen(v, "dy", &dy);
  rhoban_utils::tryReadEigen(v, "sheet_center", &sheet_center);
  rhoban_utils::tryReadVector(v, "markers_ids", &markers_ids);
}

std::string TagsSheet::getClassName() const
{
  return "tags_sheet";
}
