#include "RobotController.hpp"

#include "RhIO.hpp"

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace Vision {
namespace Localisation {

double RobotController::posExploration = 0.05;
double RobotController::angleExploration = 1;

Eigen::MatrixXd RobotController::posLimits(2, 2);

int RobotController::nbSamples = 10000;

RobotController::RobotController() : Controller() {
  setPosExploration(posExploration);
}

RobotController::RobotController(const Point &move, const Angle &rotation,
                                 double noiseGain_)
  : Controller(), ctrlMove(move), ctrlRot(rotation), noiseGain(noiseGain_)
{
  setPosExploration(posExploration * noiseGain);
}

void RobotController::setPosExploration(double newPosE) {
  posExploration = newPosE;
  for (int dim : {0, 1}) {
    posLimits(dim, 0) = -newPosE;
    posLimits(dim, 1) = newPosE;
  }
}

void RobotController::bindWithRhIO() {
  RhIO::Root.newChild("/localisation/field/RobotController");
  RhIO::Root.newFloat("/localisation/field/RobotController/angleExploration")
      ->defaultValue(angleExploration)
      ->minimum(0.0)
      ->maximum(360.0)
      ->comment("StdDev of angular exploration for azimuth in degrees/second");
  RhIO::Root.newFloat("/localisation/field/RobotController/posExploration")
      ->defaultValue(posExploration)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("StdDev of position exploration in m/second");
}

void RobotController::importFromRhIO() {
  double oldPosExpl(posExploration);
  angleExploration =
      RhIO::Root.getValueFloat(
                     "/localisation/field/RobotController/angleExploration")
          .value;
  posExploration =
      RhIO::Root.getValueFloat(
                     "/localisation/field/RobotController/posExploration")
          .value;
  if (oldPosExpl != posExploration) {
    setPosExploration(posExploration);
  }
}

void RobotController::move(FieldPosition &p, double elapsedTime) {
  p.move(ctrlMove);
  p.rotate(ctrlRot);
}

void RobotController::explore(FieldPosition &p, double elapsedTime) {
  std::uniform_real_distribution<double> angleDistrib(-angleExploration * noiseGain,
                                                      angleExploration * noiseGain);
  Eigen::VectorXd explMove =
    elapsedTime * rhoban_random::getUniformSamples(posLimits, 1, &engine)[0];
  Point explPos(explMove(0), explMove(1));
  p.move(explPos);
  p.rotate(elapsedTime * Angle(angleDistrib(engine)));
}
}
}
