#include "Filters/Custom/FieldBorderData.hpp"
#include "Filters/Custom/FieldBorder.hpp"
#include <math.h>

#define FBPRINT_DEBUG(str...) if (debug_output) { printf("[CLIPPING] "); printf(str); }

using namespace rhoban_utils;

namespace Vision {
namespace Filters {
  
FieldBorderData::FieldBorderData() {
  loc_active = true;
  reset();
}

void FieldBorderData::computeTransformations(Vision::Utils::CameraState * cs,
					     bool final_compute) {
  world_lines.clear();
  self_lines.clear();
  observation_valid = final_compute;
  try {
    for (size_t k = 0; k < pix_lines.size(); k++) {
      cv::Point2f A = pix_lines[k].first;
      cv::Point2f B = pix_lines[k].second;
      auto pointA = cs->robotPosFromImg(A.x, A.y, 1, 1, false);
      auto pointB = cs->robotPosFromImg(B.x, B.y, 1, 1, false);
      world_lines.push_back(std::pair<cv::Point2f, cv::Point2f >(pointA, pointB));
      pointA = cs->getPosInSelf(pointA);
      pointB = cs->getPosInSelf(pointB);
      self_lines.push_back(std::pair<cv::Point2f, cv::Point2f >(pointA, pointB));
      if (line_quality[k] > max_obs_score) observation_valid = false;
    }
    
    if (pix_lines.size() == 1) {
      define_segment(final_compute);
      return;
    }
        
    if (hasCorner) {
      world_corner = cs->robotPosFromImg(pix_corner.x,
					 pix_corner.y, 1, 1, false);
      self_corner = cs->getPosInSelf(world_corner);
      corner_robot_dist = cv::norm(self_corner);
      
      if (self_lines.size() >= 2) {
	// Le secteur angulaire est vu de haut, l'angle orienté ACB est
	// donc dans le sens trigo.
	// Calcul de l'angle par AlKashi
	cv::Point2f A = self_lines[0].first;
	cv::Point2f B = self_lines[1].second;
	float CA = cv::norm(A-self_corner);
	float CB = cv::norm(B-self_corner);
	float AB = cv::norm(B-A);
	if (final_compute) {
          FBPRINT_DEBUG("distances : CA=%f CB=%f AB=%f dist(corner)=%f\n", CA, CB, AB, cv::norm(self_corner));
        }
	if (CA != 0 && CB != 0) {
	  float co = (CA*CA + CB*CB - AB*AB)/ (2*CA*CB);
	  if (co < -1.0) co = -1.0; if (co > 1.0) co = 1.0;
	  corner_angle = acos(co);
	  if (final_compute) {
            FBPRINT_DEBUG("Corrected Corner angle (ACB) = %f\n", 180.0 / M_PI * corner_angle);
          }
	  if (fabs(M_PI/2 - corner_angle) > deg2rad(tolerance_angle_corner) // TODO: parametre
              || CA < minimal_segment_length || CB < minimal_segment_length ) {
	    if (final_compute) {
              FBPRINT_DEBUG("Corner avoided (bad angle or segment too short)\n");
            }
	    hasCorner = false;
            observation_valid = false;
	  }

          if (corner_robot_dist > max_dist_corner) {
	    if (final_compute) { FBPRINT_DEBUG("Corner avoided : it is too far : seen at %f\n", corner_robot_dist); }
            observation_valid = false;
	  }

	  if (fabs(M_PI - corner_angle) < deg2rad(tolerance_angle_line)) {
	    define_segment(final_compute);
            return;
	  }
	} else {
	  hasCorner = false;
          observation_valid = false;
	  if (final_compute) { FBPRINT_DEBUG("Segment are too short\n"); }
	}
      }
      else {
	hasCorner = false;
	if (final_compute) { FBPRINT_DEBUG("There is only one line\n"); }
      }
    }
    else {
      if (final_compute) { FBPRINT_DEBUG("No corner\n"); }
      observation_valid = false;
    }
  }
  catch ( const std::exception & e ) {
    // std::cerr << e.what();
    if (final_compute) { FBPRINT_DEBUG("CBB: exception during computation\n"); }
    rollback_computation();
  }
}

void FieldBorderData::define_segment(bool debug_info) {
  hasCorner = false;
  pix_segment.first = pix_lines[0].first;
  pix_segment.second = pix_lines[pix_lines.size()-1].second;
  world_segment.first = world_lines[0].first;
  world_segment.second = world_lines[world_lines.size()-1].second;  
  self_segment.first = self_lines[0].first;
  self_segment.second = self_lines[self_lines.size()-1].second;
  double len = cv::norm(self_segment.second - self_segment.first);

  cv::Point2f AB = self_segment.first-self_segment.second;
  double dist_AB = cv::norm(AB);
  if (dist_AB == 0) return;
  // TODO: transferer ce calcul dans brut_data, c'est pareil pour chaque particule...
  cv::Point2f AB_unit = (1.0 / dist_AB) * AB;
  cv::Point2f AB_ortho_unit(-AB_unit.y, AB_unit.x);
  double dot_bot_seg = AB_ortho_unit.dot(self_segment.first); 
  double dist_bot_seg = fabs(dot_bot_seg);

  double min_len = 0.50;
  double max_dist = 4.0;

  // Note: parametre on ne regarde pas les segments en dessous de 'min_len'
  if (len > min_len && dist_bot_seg < max_dist) {
    if (debug_info) {
      FBPRINT_DEBUG("Found Segment (%0.2f,%0.2f) -> (%0.2f,%0.2f) (len = %0.2f)\n",
                    self_segment.first.x,
                    self_segment.first.y,
                    self_segment.second.x,
                    self_segment.second.y,
                    len);
    }
    hasSegment = true;
    observation_valid &= true;
  }
  else {
    if (len <= min_len) {
      FBPRINT_DEBUG("segment too short\n");
    }
    if (dist_bot_seg >= max_dist) {
      FBPRINT_DEBUG("segment too far (at dist %0.2f m)\n", dist_bot_seg);
    }
  }
    
}

std::pair<cv::Point2f, cv::Point2f >
FieldBorderData::getSegmentInSelf() const {
  return self_segment;
}

std::pair<cv::Point2f, cv::Point2f>
FieldBorderData::getSegmentInWorld() const {
  return world_segment;
}
  
float FieldBorderData::getRobotCornerDist() {
  return corner_robot_dist;
}
  
void FieldBorderData::addPixCorner(cv::Point2f C) {
  hasCorner = true;
  pix_corner = C;
}
  
cv::Point2f FieldBorderData::getPixCorner() const {
  return pix_corner;
}

cv::Point2f FieldBorderData::getCornerInWorldFrame() const {
  return world_corner;
}

cv::Point2f FieldBorderData::getCornerInSelf() const {
  return self_corner;
}

std::vector<std::pair<cv::Point2f, cv::Point2f > > FieldBorderData::getLinesInSelf() const {
  return self_lines;
}
float FieldBorderData::getCornerAngle() const {
  return corner_angle;
}
  
std::vector<std::pair<cv::Point2f, cv::Point2f > > FieldBorderData::getLinesInWorldFrame() const {
  return world_lines;
}

void FieldBorderData::clearLines() {
  pix_lines.clear();
  line_quality.clear();
  world_lines.clear();
  self_lines.clear();
  line_scores.clear();
}

void FieldBorderData::reset() {
  observation_valid = false;
  hasSegment = false;
  hasCorner = false;
  corner_robot_dist = -1;
  corner_angle = -1;
  clearLines();
}

void FieldBorderData::rollback_computation() {
  hasSegment = false;
  hasCorner = false;
  corner_robot_dist = -1;
  corner_angle = -1;
  world_lines.clear();
  self_lines.clear();
  observation_valid = false;
}

std::vector<std::pair<cv::Point2f, cv::Point2f > > FieldBorderData::getPixLines() const {
  return pix_lines;
}
  
void FieldBorderData::pushPixLine(double quality, std::pair<cv::Point2f,cv::Point2f> L) {
  if (cv::norm(L.first-L.second) > 0) {
    pix_lines.push_back(L);
    line_quality.push_back(quality);
  }
}

bool FieldBorderData::is_obs_valid() {
  return loc_active && observation_valid;
}
  
}
}
