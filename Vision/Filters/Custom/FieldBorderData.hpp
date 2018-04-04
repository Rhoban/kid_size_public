#ifndef VISION_FILTERS_FIELDBORDERDATA_HPP
#define VISION_FILTERS_FIELDBORDERDATA_HPP
#include "Filters/Filter.hpp"
#include "CameraState/CameraState.hpp"

namespace Vision {
namespace Filters {

/* the data exchange class between vision filter and 
 * localisation stuff */
class FieldBorderData {
private:
  std::vector<std::pair<cv::Point2f, cv::Point2f > > pix_lines;
  std::vector<std::pair<cv::Point2f, cv::Point2f > > world_lines;
  std::vector<std::pair<cv::Point2f, cv::Point2f > > self_lines;
  std::vector<double> line_quality;
  cv::Point2f pix_corner;
  cv::Point2f world_corner;
  cv::Point2f self_corner;

  float corner_robot_dist; // (m)
  float corner_angle; // (rad)

  std::pair<cv::Point2f, cv::Point2f> pix_segment;
  std::pair<cv::Point2f, cv::Point2f> world_segment;
  std::pair<cv::Point2f, cv::Point2f> self_segment; 

  bool observation_valid;
public:
  double max_obs_score;
  bool loc_active;
  bool debug_output;
  bool is_obs_valid();
  double max_dist_corner;
  double tolerance_angle_corner;
  double tolerance_angle_line;
  double minimal_segment_length;
 
  std::vector<float> line_scores;
  FieldBorderData();

  // The distance between the corner and the robot (in robot self)
  float getRobotCornerDist();
  
  // TODO: ajouter la qualité, et le point d'intersection
  bool hasCorner;
  bool hasSegment;
  
  void computeTransformations(Vision::Utils::CameraState * cs,
			      bool final_compute);
  void rollback_computation();
  void addPixCorner(cv::Point2f C);
  void reset();
  void clearLines();
  void pushPixLine(double quality, std::pair<cv::Point2f,cv::Point2f> L);
  cv::Point2f getPixCorner() const;
  cv::Point2f getCornerInWorldFrame() const;
  std::vector<std::pair<cv::Point2f, cv::Point2f > > getLinesInWorldFrame() const;
  cv::Point2f getCornerInSelf() const;
  std::vector<std::pair<cv::Point2f, cv::Point2f > > getLinesInSelf() const;
  std::vector<std::pair<cv::Point2f, cv::Point2f > > getPixLines() const;
  std::pair<cv::Point2f, cv::Point2f> getSegmentInSelf() const;
  std::pair<cv::Point2f, cv::Point2f> getSegmentInWorld() const;
  float getCornerAngle() const;
  void define_segment(bool debug_info = false);
};





}
}
#endif 
