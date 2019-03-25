#include "CalibrationSet.hpp"

#include <rhoban_utils/util.h>

CalibrationSet::CalibrationSet()
  : marker_size(0.09)
  , robot_pos(-0.14, 0.065)
  , panel_size(0.63, 0.315)
  , panel_thickness(0.003)
  , border_to_center_1(0.185)
  , border_to_center_2(0.15)
  , sheet_dx(0.099)
  , sheet_dy(0.099)
  , sheets_spacing(0.3)
{
  updateSheets();
  getMarkers();
}

void CalibrationSet::updateSheets()
{
  sheets.clear();

  for (int panel_idx = 0; panel_idx < 10; panel_idx++)
  {
    Eigen::Vector3d dx = getXDir(panel_idx) * sheet_dx;
    Eigen::Vector3d dy = getYDir(panel_idx) * sheet_dy;
    Eigen::Vector3d sheet1 = sheet1Center(panel_idx);
    std::vector<int> markers1 = getMarkersIndices(panel_idx, 0);
    sheets.push_back(TagsSheet(marker_size, dx, dy, sheet1, markers1));
    if (panel_idx != 9)
    {
      Eigen::Vector3d sheet2 = sheet2Center(panel_idx);
      std::vector<int> markers2 = getMarkersIndices(panel_idx, 1);
      sheets.push_back(TagsSheet(marker_size, dx, dy, sheet2, markers2));
    }
  }
}

std::map<int, ArucoTag> CalibrationSet::getMarkers() const
{
  std::map<int, ArucoTag> dictionary;
  for (const TagsSheet& sheet : sheets)
  {
    for (const std::pair<int, ArucoTag>& entry : sheet.getMarkers())
    {
      if (dictionary.count(entry.first))
      {
        throw std::logic_error(DEBUG_INFO + " duplicated entry for id: " + std::to_string(entry.first));
      }
      dictionary[entry.first] = entry.second;
    }
  }
  return dictionary;
}

Json::Value CalibrationSet::toJson() const
{
  Json::Value v;
  v["marker_size"] = marker_size;
  v["panel_thickness"] = panel_thickness;
  v["border_to_center_1"] = border_to_center_1;
  v["border_to_center_2"] = border_to_center_2;
  v["sheet_dx"] = sheet_dx;
  v["sheet_dy"] = sheet_dy;
  v["sheets_spacing"] = sheets_spacing;
  v["panel_size"] = rhoban_utils::vector2Json(panel_size);
  v["robot_pos"] = rhoban_utils::vector2Json(robot_pos);
  return v;
}

void CalibrationSet::fromJson(const Json::Value& v, const std::string& dir_path)
{
  (void)dir_path;
  rhoban_utils::tryRead(v, "marker_size", &marker_size);
  rhoban_utils::tryRead(v, "panel_thickness", &panel_thickness);
  rhoban_utils::tryRead(v, "border_to_center_1", &border_to_center_1);
  rhoban_utils::tryRead(v, "border_to_center_2", &border_to_center_2);
  rhoban_utils::tryRead(v, "sheet_dx", &sheet_dx);
  rhoban_utils::tryRead(v, "sheet_dy", &sheet_dy);
  rhoban_utils::tryRead(v, "sheets_spacing", &sheets_spacing);
  rhoban_utils::tryReadEigen(v, "panel_size", &panel_size);
  rhoban_utils::tryReadEigen(v, "robot_pos", &robot_pos);
}

std::string CalibrationSet::getClassName() const
{
  return "CalibrationSet";
}

Eigen::Vector3d CalibrationSet::getXDir(int panel_idx) const
{
  switch (panel_idx)
  {
    case 1:
    case 2:  // left panels
      return Eigen::Vector3d(1, 0, 0);
    case 3:
    case 4:
    case 5:  // front panels
      return Eigen::Vector3d(0, -1, 0);
    case 6:
    case 7:  // right panels
      return Eigen::Vector3d(-1, 0, 0);
    default:  // horizontal panels
      return Eigen::Vector3d(0, -1, 0);
  }
}

Eigen::Vector3d CalibrationSet::getYDir(int panel_idx) const
{
  if (panel_idx >= 1 && panel_idx <= 7)
  {  // vertical panels
    return Eigen::Vector3d(0, 0, -1);
  }
  // horizontal panels
  return Eigen::Vector3d(-1, 0, 0);
}

Eigen::Vector3d CalibrationSet::sheet1Center(int panel_idx) const
{
  // Explicit naming
  double panel_length = panel_size(0);
  double panel_width = panel_size(1);
  // Non obvious positions
  double x_horizontal = panel_length / 2 - border_to_center_1;
  double z_vertical = panel_length - border_to_center_2;
  // First extracting world pos (ref is center of P9)
  Eigen::Vector3d world_pos;
  switch (panel_idx)
  {
    case 0:
      world_pos = Eigen::Vector3d(x_horizontal, panel_width, panel_thickness);
      break;
    case 1:
      world_pos = Eigen::Vector3d(-panel_width / 2, 3 * panel_width / 2, z_vertical);
      break;
    case 2:
      world_pos = Eigen::Vector3d(panel_width / 2, 3 * panel_width / 2, z_vertical);
      break;
    case 3:
      world_pos = Eigen::Vector3d(panel_length / 2, panel_width, z_vertical);
      break;
    case 4:
      world_pos = Eigen::Vector3d(panel_length / 2, 0, z_vertical);
      break;
    case 5:
      world_pos = Eigen::Vector3d(panel_length / 2, -panel_width, z_vertical);
      break;
    case 6:
      world_pos = Eigen::Vector3d(panel_width / 2, -3 * panel_width / 2, z_vertical);
      break;
    case 7:
      world_pos = Eigen::Vector3d(-panel_width / 2, -3 * panel_width / 2, z_vertical);
      break;
    case 8:
      world_pos = Eigen::Vector3d(x_horizontal, -panel_width, panel_thickness);
      break;
    case 9:
      world_pos = Eigen::Vector3d(x_horizontal, 0, panel_thickness);
      break;
  }
  // Modification for non zero robot position
  Eigen::Vector3d pos_in_self = world_pos;
  pos_in_self.segment(0, 2) -= robot_pos;

  return pos_in_self;
}

Eigen::Vector3d CalibrationSet::sheet2Center(int panel_idx) const
{
  return sheet1Center(panel_idx) + getYDir(panel_idx) * sheets_spacing;
}

std::vector<int> CalibrationSet::getMarkersIndices(int panel_idx, int sheet_idx)
{
  std::vector<int> indices;
  for (int i = 0; i < 6; i++)
  {
    int marker_id = panel_idx * 12 + sheet_idx * 6 + i;
    indices.push_back(marker_id);
  }
  return indices;
}
