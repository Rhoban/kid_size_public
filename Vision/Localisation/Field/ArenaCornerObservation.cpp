#include "ArenaCornerObservation.hpp"
#include <math.h>
#include "robocup_referee/constants.h"
#include "Field/Field.hpp"
#include "CameraState/CameraState.hpp"
#include "Filters/Custom/FieldBorderData.hpp"
#include <vector>
#include "RhIO.hpp"

#include "rhoban_utils/logging/logger.h"
static rhoban_utils::Logger out("ArenaCornerObservation");

using Vision::Utils::CameraState;
using robocup_referee::Constants;
using namespace rhoban_utils;

// double getMinScore() const override; à implémenter
// les observations elle naviguent entre pError et 1.0

namespace Vision {
namespace Localisation {

double ArenaCornerObservation::maxAngleError = 30;
double ArenaCornerObservation::sigmoidOffset = 0.6;
double ArenaCornerObservation::sigmoidLambda = 5;

static double arenaLength = Constants::field.fieldLength + 2*Constants::field.borderStripWidth;
static double arenaWidth = Constants::field.fieldWidth + 2*Constants::field.borderStripWidth;

  
bool ArenaCornerObservation::debug = false;

ArenaCornerObservation::ArenaCornerObservation() {}

ArenaCornerObservation::ArenaCornerObservation(const Vision::Filters::FieldBorderData & brut_data_,
                                               const Angle &panToArenaCorner,
                                               const Angle &tiltToArenaCorner,
                                               double robotHeight_,
                                               double weight_) {
  brut_data = brut_data_;
  pan = panToArenaCorner;
  tilt = tiltToArenaCorner;
  robotHeight = robotHeight_;
  weight = weight_;
  sign_dot = 1;
  seg_dir = 0;
  
  if (debug) {
    std::ostringstream oss;
    oss << "Creating obs: pan = " << pan.getSignedValue()
        << ", tilt = " << tilt.getSignedValue();
    out.log(oss.str().c_str());
  }

  if (brut_data.hasCorner) {
    auto C = brut_data.getCornerInSelf();
    auto lines = brut_data.getLinesInSelf(); 
    auto A = lines[0].first;
    auto B = lines[1].second;
    cv::Point2f e1 = A-C;
    double e1_len = cv::norm(e1);
    if (e1_len==0)
      throw std::string("ArenaCornerObservation e1_len null");
    e1 = (1.0 / e1_len) * e1;
    cv::Point2f e2(-e1.y, e1.x);
    cv::Point2f bot_in_corner_frm(e1.dot(-C), e2.dot(-C));
    if (bot_in_corner_frm.x == 0 && bot_in_corner_frm.y == 0)
      throw std::string("ArenaCornerObservation bot on the corner (TODO)");
    double bot_arg_in_corner_frm = atan2(bot_in_corner_frm.y, bot_in_corner_frm.x);
    double corner_arg_in_bot = atan2(C.y, C.x);
    Angle bot_orient = Angle(rad2deg(bot_arg_in_corner_frm + M_PI - corner_arg_in_bot));

    candidates.clear();

    FieldPosition br_corner_pos(-arenaLength/2 + bot_in_corner_frm.x,
				-arenaWidth/2 + bot_in_corner_frm.y,
				bot_orient.getSignedValue());
    candidates.push_back(br_corner_pos);
    FieldPosition tr_corner_pos(arenaLength/2 - bot_in_corner_frm.y,
				-arenaWidth/2 + bot_in_corner_frm.x,
				(bot_orient + Angle(90)).getSignedValue());
    candidates.push_back(tr_corner_pos);
    FieldPosition tl_corner_pos(arenaLength/2 - bot_in_corner_frm.x,
				arenaWidth/2 - bot_in_corner_frm.y,
				(bot_orient + Angle(180)).getSignedValue());
    candidates.push_back(tl_corner_pos);
    FieldPosition bl_corner_pos(-arenaLength/2 + bot_in_corner_frm.y,
				arenaWidth/2 - bot_in_corner_frm.x,
				(bot_orient + Angle(270)).getSignedValue());
    candidates.push_back(bl_corner_pos);   
  }
  else if (brut_data.hasSegment) {
    float default_potential = pError;
    auto seg_in_self = brut_data.getSegmentInSelf();
    cv::Point2f A = seg_in_self.first;
    cv::Point2f B = seg_in_self.second;
    cv::Point2f AB = B-A;
    double dist_AB = cv::norm(AB);
    if (dist_AB == 0)
      throw std::string("ArenaCornerObservation dist_AB = 0");
    cv::Point2f AB_unit = (1.0 / dist_AB) * AB;
    cv::Point2f AB_ortho_unit(-AB_unit.y, AB_unit.x);
    double dot_bot_seg = AB_ortho_unit.dot(A);//[m]
    double dist_bot_seg = fabs(dot_bot_seg);
    sign_dot = (dot_bot_seg >= 0) ? 1 : -1;
    seg_dir = rad2deg(atan2(AB.y, AB.x));
    
    // On peut faire mieux, les position et les angles sont liés...
    x_candidates.clear();
    x_candidates.push_back(arenaLength/2 - dist_bot_seg);
    x_candidates.push_back(-arenaLength/2 + dist_bot_seg);
    y_candidates.clear();
    y_candidates.push_back(arenaWidth/2 - dist_bot_seg);
    y_candidates.push_back(-arenaWidth/2 + dist_bot_seg);
  }
  else {
    throw std::string("ArenaCornerObservation no corner neither segment");
  }
}

Angle ArenaCornerObservation::getPan() const { return pan; }
Angle ArenaCornerObservation::getTilt() const { return tilt; }
double ArenaCornerObservation::getWeight() const { return weight; }

Vision::Filters::FieldBorderData ArenaCornerObservation::getBrutData() {
  return brut_data;
}

double ArenaCornerObservation::pError = 0.3;
double ArenaCornerObservation::potential_pos50 = 0.1;//[m]
double ArenaCornerObservation::potential_angle50 = 30;
double ArenaCornerObservation::potential_exp = 1;
  
double ArenaCornerObservation::basic_pot(double x, double x_half) const {
  return pError + (1-pError) * exp(-pow(fabs(x/x_half),20)*(-log(0.5-pError)));
}

double ArenaCornerObservation::potential(const FieldPosition & candidate,
                                         const FieldPosition & p) const {
  double pos_error =(p.getRobotPosition()- candidate.getRobotPosition()).getLength();// [m]
  double angle_error = fabs((candidate.getOrientation() -
                             p.getOrientation()).getSignedValue()); /* deg */
  double pos_pot = basic_pot(pos_error, potential_pos50);
  double angle_pot = basic_pot(angle_error, potential_angle50);

  return sqrt(pos_pot * angle_pot);
}

double ArenaCornerObservation::corner_potential(const FieldPosition &p) const {
  // one takes the maximum potential
  // regarding all the candidates
  double best_one = 0;
  for (auto c : candidates) {
    float c_pot = potential(c, p);
    if (c_pot > best_one) best_one = c_pot;
  }
  return best_one;
}

double ArenaCornerObservation::segment_potential(const FieldPosition &p) const {

  // Note: one assumes that the robot is within the field, and so
  // the viewed segment goes in the anti-trigonometic way.

  double best_one_x = 0;
  for (auto x : x_candidates) {
    int sign_x = (x>=0) ? 1 : -1;
    double pos_err = fabs(p.getRobotPosition().x-x);
    double pos_pot = basic_pot(pos_err, potential_pos50);
    //     printf("pos_err = %f -> %f\n", pos_err, pos_pot);
    double base_angle = -sign_dot * sign_x * 90;
    double angle_error = fabs((Angle(base_angle - seg_dir) -
			       p.getOrientation()).getSignedValue()); /* deg */
    double angle_pot = basic_pot(angle_error, potential_angle50);
    double pot = sqrt(pos_pot * angle_pot);
    if (best_one_x < pot) best_one_x = pot;
  }

  // ----

  double best_one_y = 0;
  for (auto y : y_candidates) {
    int sign_y = (y>=0) ? 1 : -1;
    double pos_err = fabs(p.getRobotPosition().y-y);
    double pos_pot = basic_pot(pos_err, potential_pos50);
    double base_angle = 90 - sign_dot * sign_y * 90;
    double angle_error = fabs((Angle(base_angle - seg_dir) -
			       p.getOrientation()).getSignedValue()); /* deg */
    double angle_pot = basic_pot(angle_error, potential_angle50);
    double pot = sqrt(pos_pot * angle_pot);
    if (best_one_y < pot) best_one_y = pot;
  }
    
  // TODO: on peut faire mieux en regardant les extremités du segment
  // et en les positionnant dans le terrain

  return max(best_one_x, best_one_y);
}

std::string ArenaCornerObservation::toStr() const {
  if (brut_data.hasCorner) {
    return "Corner Observed";
  }
  else if (brut_data.hasSegment) {
    return "Segment Observed";
  }
  else {
    return "Unconsistant observation";
  }
}
  
double ArenaCornerObservation::potential(const FieldPosition &p) const {

  if (brut_data.hasCorner) {
    return corner_potential(p);
  }
  else if (brut_data.hasSegment) {
    return segment_potential(p);
  } 
  else {
    return pError;
  }
}

void ArenaCornerObservation::bindWithRhIO() {
  RhIO::Root.newFloat("/localisation/field/ArenaCornerObservation/pError")
      ->defaultValue(pError)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("The false positive probability");
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaCornerObservation/maxAngleError")
      ->defaultValue(maxAngleError)
      ->minimum(0.0)
      ->maximum(180)
      ->comment(
            "The maximum angle difference between expectation and observation");
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaCornerObservation/sigmoidOffset")
      ->defaultValue(sigmoidOffset)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment(
            "The value at which dScore/dx is lambda, with dx = dAngle/maxAngle");
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaCornerObservation/sigmoidLambda")
      ->defaultValue(sigmoidLambda)
      ->minimum(0.0)
      ->maximum(1000.0)
      ->comment("Cf. sigmoidOffset");
  RhIO::Root.newBool("/localisation/field/ArenaCornerObservation/debug")
      ->defaultValue(debug)
      ->comment("Print message on observation creation");
}

void ArenaCornerObservation::importFromRhIO() {
  RhIO::IONode &node =
      RhIO::Root.child("localisation/field/ArenaCornerObservation");
  pError = node.getValueFloat("pError").value;
  maxAngleError = node.getValueFloat("maxAngleError").value;
  sigmoidOffset = node.getValueFloat("sigmoidOffset").value;
  sigmoidLambda = node.getValueFloat("sigmoidLambda").value;
  debug = node.getValueBool("debug").value;
}

std::string ArenaCornerObservation::getClassName() const {
  return "ArenaCornerObservation";
}

Json::Value ArenaCornerObservation::toJson() const {
  Json::Value v;
  v["robotHeight"] = robotHeight;
  v["pan"] = pan.getSignedValue();
  v["tilt"] = tilt.getSignedValue();
  return v;
}

void ArenaCornerObservation::fromJson(const Json::Value & v, const std::string & dir_name) {
  rhoban_utils::tryRead(v,"robotHeight",&robotHeight);
  rhoban_utils::tryRead(v,"pan",&pan);
  rhoban_utils::tryRead(v,"tilt",&tilt);
}

double ArenaCornerObservation::getMinScore() const {
  return pError + 0.05;
}
}
}
