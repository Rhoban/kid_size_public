#include "ArenaBorderObservation.hpp"

#include "Field/Field.hpp"
#include "CameraState/CameraState.hpp"

#include "RhIO.hpp"

#include "rhoban_utils/logging/logger.h"

#include "rhoban_random/tools.h"

#include "Utils/Interface.h"

static rhoban_utils::Logger out("ArenaBorderObservation");

using Vision::Utils::CameraState;

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace Vision {
namespace Localisation {

double ArenaBorderObservation::pError = 0.1;

double ArenaBorderObservation::lAngleMaxError = 10;
double ArenaBorderObservation::lSigmoidOffset = 0.6;
double ArenaBorderObservation::lSigmoidLambda = 5;

double ArenaBorderObservation::cpAngleMaxError = 10;
double ArenaBorderObservation::cpSigmoidOffset = 0.6;
double ArenaBorderObservation::cpSigmoidLambda = 5;

bool ArenaBorderObservation::debug = false;

int ArenaBorderObservation::maxTries = 10;
double ArenaBorderObservation::minDist = 0.2; //[m]

ArenaBorderObservation::ArenaBorderObservation() {}

ArenaBorderObservation::ArenaBorderObservation(
    const ParametricLine &imgSeenLine, int imgWidth, int imgHeight,
    CameraState &cs) {
  if (debug) {
    std::ostringstream oss;
    oss << "Creating an observation with seenLine : [rho="
        << imgSeenLine.getRho() << ",theta=" << imgSeenLine.getTheta() << "]"
        << ", imgSize: " << imgWidth << "x" << imgHeight;
    out.log(oss.str().c_str());
  }
  std::vector<ParametricLine> imgBorders;
  imgBorders.push_back(ParametricLine(Point(0, 0), Point(imgWidth - 1, 0)));
  imgBorders.push_back(ParametricLine(Point(0, 0), Point(0, imgHeight - 1)));
  imgBorders.push_back(ParametricLine(Point(imgWidth - 1, imgHeight - 1),
                                      Point(imgWidth - 1, 0)));
  imgBorders.push_back(ParametricLine(Point(imgWidth - 1, imgHeight - 1),
                                      Point(0, imgHeight - 1)));
  // Determining the extremum positions of the line on the image
  std::vector<Point> extremums;
  for (const auto &line : imgBorders) {
    try {
      Point p = imgSeenLine.intersection(line);
      if (p.x >= 0 && p.x <= imgWidth - 1 && p.y >= 0 && p.y <= imgHeight - 1) {
        extremums.push_back(p);
      }
    } catch (const CollinearException &exc) {
    }
  }
  if (extremums.size() != 2) {
    // TODO is it possible for this to happen? To check
    throw std::runtime_error(
        "Failed to find extremums of lines in ArenaBorder");
  }

  //      std::cout << "--- Images Extremums ---" << std::endl
  //                << "\tExt0: " << extremums[0] << std::endl
  //                << "\tExt1: " << extremums[1] << std::endl;
  //

  // Generating random points inside the image line, and ensuring that the
  // distance between them in the robot basis is higher than minDist
  Point src = extremums[0];
  Point diff = extremums[1] - src;
  std::vector<Point> points;
  std::uniform_real_distribution<double> rndDiff(0, 1);
  auto generator = rhoban_random::getRandomEngine();
  int nbTries = 0;
  while (nbTries < maxTries && points.size() < 2) {
    Point pImg = src + diff * rndDiff(generator);
    try {
      // Converting to centimeters
      cv::Point2f p_2f = cs.robotPosFromImg(pImg.x, pImg.y, imgWidth, imgHeight);
      Point p = 100 * cv2rg(p_2f);
      // std::cerr<<"DEBUG point; "<<pImg.x<<" "<<pImg.y<<" "<<imgWidth<<"
      // "<<imgHeight<<" "<<p.x<<" "<<p.y<<std::endl;
      if (points.size() == 0 || points[0].getDist(p) > minDist) {
        points.push_back(p);
      }
    } catch (const std::runtime_error &exc) {
      std::cerr << "In ArenaBorderObservation: " << exc.what() << std::endl;
    }
    nbTries++;
  }
  if (points.size() != 2) {
    std::ostringstream oss;
    oss << "Failed to obtain 2 points after " << maxTries << " tries";
    throw std::runtime_error(oss.str());
  }

  robotSeenLine = ParametricLine(points[0], points[1]);
  robotClosestPoint = robotSeenLine.projectPoint(Point(0, 0));
  robotHeight = cs.getHeight();

  // std::cout << "--- Creating ArenaBorderObservation ---" << std::endl
  //          << "\tPoint0: points[0]: " << points[0] << std::endl
  //          << "\tPoint1: points[1]: " << points[1] << std::endl
  //          << "\trobotClosestPoint: " << robotClosestPoint << std::endl
  //          << "\trobotSeenLine: " << robotSeenLine << std::endl;
}

double
ArenaBorderObservation::angleScore(const FieldPosition &p,
                                   const ParametricLine &theoricLine) const {
  Angle expectedAngle = theoricLine.getTheta() - p.getOrientation();
  Angle diffAngle = expectedAngle - robotSeenLine.getTheta();
  if (std::fabs(diffAngle.getSignedValue()) > 90) {
    diffAngle = diffAngle + Angle(180);
  }
  double absAngleDiff = std::fabs(diffAngle.getSignedValue());
  if (absAngleDiff > lAngleMaxError)
    return 0.0;
  double score = sigmoidScore(absAngleDiff, lAngleMaxError, 0.0, lSigmoidOffset,
                              lSigmoidLambda);
  return score;
}

double ArenaBorderObservation::closestPointScore(
    const FieldPosition &p, const ParametricLine &theoricLine) const {
  Point fieldThClosestPoint = theoricLine.projectPoint(p.getRobotPosition());
  Point robotThClosestPoint = (fieldThClosestPoint - p.getRobotPosition())
                                  .rotation(-p.getOrientation());

  Point seenPos = robotClosestPoint;
  Point expPos = robotThClosestPoint;
  // DEBUG
  // std::cerr<<"DEBUG BORDER point: "<<seenPos<<" "<<expPos<<std::endl;
  cv::Point3f seenDir(seenPos.x, seenPos.y, -robotHeight);
  cv::Point3f expectedDir(expPos.x, expPos.y, -robotHeight);
  Angle aDiff = angleBetween(seenDir, expectedDir);
  // Parameters of scoring
  if (aDiff.getValue() > cpAngleMaxError)
    return 0;
  return sigmoidScore(aDiff.getValue(), cpAngleMaxError, 0.0, cpSigmoidOffset,
                      cpSigmoidLambda);
}

double ArenaBorderObservation::potential(const FieldPosition &p) const {
  double bestScore = 0;
  for (const auto &thLine : Field::Field::getArenaBorders()) {
    double score = 1.0;
    score *= angleScore(p, thLine);
    score *= closestPointScore(p, thLine);
    if (score > bestScore) {
      bestScore = score;
    }
  }
  return bestScore * (1 - pError) + pError;
}

void ArenaBorderObservation::bindWithRhIO() {
  RhIO::Root.newFloat("/localisation/field/ArenaBorderObservation/pError")
      ->defaultValue(pError)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("The false positive probability");
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/lAngleMaxError")
      ->defaultValue(lAngleMaxError)
      ->minimum(0.0)
      ->maximum(180);
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/lSigmoidOffset")
      ->defaultValue(lSigmoidOffset)
      ->minimum(0.0)
      ->maximum(1.0);
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/lSigmoidLambda")
      ->defaultValue(lSigmoidLambda)
      ->minimum(0.0)
      ->maximum(1000.0)
      ->comment("Cf. sigmoidOffset");
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/cpAngleMaxError")
      ->defaultValue(cpAngleMaxError)
      ->minimum(0.0)
      ->maximum(180);
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/cpSigmoidOffset")
      ->defaultValue(lSigmoidOffset)
      ->minimum(0.0)
      ->maximum(1.0);
  RhIO::Root.newFloat(
                 "/localisation/field/ArenaBorderObservation/cpSigmoidLambda")
      ->defaultValue(lSigmoidLambda)
      ->minimum(0.0)
      ->maximum(1000.0);
  RhIO::Root.newInt("/localisation/field/ArenaBorderObservation/maxTries")
      ->defaultValue(maxTries)
      ->minimum(0)
      ->maximum(1000);
  RhIO::Root.newFloat("/localisation/field/ArenaBorderObservation/minDist")
      ->defaultValue(minDist)
      ->minimum(0.0)
      ->maximum(5.0)
      ->comment("The minimal distance between two points to consider the line [m]");
  RhIO::Root.newBool("/localisation/field/ArenaBorderObservation/debug")
      ->defaultValue(debug)
      ->comment("Print message on observation creation");
}

void ArenaBorderObservation::importFromRhIO() {
  RhIO::IONode &node =
      RhIO::Root.child("localisation/field/ArenaBorderObservation");
  pError = node.getValueFloat("pError").value;
  lAngleMaxError = node.getValueFloat("lAngleMaxError").value;
  lSigmoidOffset = node.getValueFloat("lSigmoidOffset").value;
  lSigmoidLambda = node.getValueFloat("lSigmoidLambda").value;
  cpAngleMaxError = node.getValueFloat("cpAngleMaxError").value;
  cpSigmoidOffset = node.getValueFloat("cpSigmoidOffset").value;
  cpSigmoidLambda = node.getValueFloat("cpSigmoidLambda").value;
  maxTries = node.getValueInt("maxTries").value;
  minDist = node.getValueFloat("minDist").value;
  debug = node.getValueBool("debug").value;
}

std::string ArenaBorderObservation::getClassName() const {
  return "ArenaBorderObservation";
}

Json::Value ArenaBorderObservation::toJson() const {
  double rho = robotSeenLine.getRho();
  double theta = robotSeenLine.getTheta().getSignedValue();
  Json::Value v;
  v["robotHeight"] = robotHeight;
  v["rho"] = rho;
  v["theta"] = theta;
  return v;
}

void ArenaBorderObservation::fromJson(const Json::Value & v, const std::string & dir_name) {
  rhoban_utils::tryRead(v,"robotHeight",&robotHeight);
  double rho(0.0), theta(0.0);
  rhoban_utils::tryRead(v,"rho",&rho);
  rhoban_utils::tryRead(v,"theta",&theta);
  robotSeenLine = ParametricLine::fromRhoTheta(rho, Angle(theta));
  robotClosestPoint = robotSeenLine.projectPoint(Point(0, 0));
}

double ArenaBorderObservation::getMinScore() const { return pError; }
}
}
