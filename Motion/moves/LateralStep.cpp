#include <math.h>
#include <services/ModelService.h>
#include <services/LocalisationService.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <Utils/Euler.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/serialization/json_serializable.h>
#include "LateralStep.hpp"

static rhoban_utils::Logger logger("lateral");
using namespace Leph;

using namespace rhoban_utils;

// DOFs
static std::vector<std::string> dofs = {
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll", "left_knee", "left_ankle_pitch", "left_ankle_roll"
};

void LateralStep::setLeft(bool left){
  bind->node().setBool("left", left);    
}

LateralStep::LateralStep()
    : lateralStepModel(Leph::SigmabanModel), 
    inv(lateralStepModel)
{
  initializeBinding();

  /*  bind->bindNew("over", over, RhIO::Bind::PushOnly)
      ->defaultValue(false)
      ->comment("Is the kick over");
  */

  bind->bindNew("live", live, RhIO::Bind::PullOnly)
    ->defaultValue(false)
    ->comment("Live tuning?");

  bind->bindNew("dt", dt, RhIO::Bind::PullOnly)
    ->defaultValue(0.005)->comment("Dt for kick gen");
  bind->bindNew("left", left, RhIO::Bind::PullOnly)
    ->defaultValue(false)->comment("Left foot");

  bind->bindFunc("LateralStepGen", "Pre-compute the lateral step",
                 &LateralStep::cmdLateralStepGen, *this);

  // Initializing inverse kinematics
  for (auto dof : dofs) {
    inv.addDOF(dof);
    }
    inv.setLowerBound("left_knee", 0.0);
    inv.setLowerBound("right_knee", 0.0);

    inv.addTargetPosition("right_foot", "right_foot_tip");
    inv.addTargetPosition("left_foot", "left_foot_tip");

    inv.addTargetOrientation("right_foot", "right_foot_tip");
    inv.addTargetOrientation("left_foot", "left_foot_tip");

    loadCompiledLateralSteps();
}
        

std::map<std::string, Function> LateralStep::loadCompiledLateralStep(std::string filename)
{
    std::map<std::string, Function> splines;
    try {
        splines = Function::fromFile(filename);
    } catch (const rhoban_utils::JsonParsingError & exc) {
        logger.error("%s", exc.what());
    }

    tMax = 0;
    for (auto &entry :splines) {
        double duration = entry.second.getXMax();
        if (duration > tMax) {
            tMax = duration;
        }
    }
    
    return splines;
}
        
void LateralStep::loadCompiledLateralSteps()
{
    splines = loadCompiledLateralStep("compiled_lateral_step.json");
}

void LateralStep::loadLateralStep(std::string filename)
{
    bind->pull();
    splines.clear();
    double t = 0;
    double xMax = 0;

    logger.log("Generating lateral step");

    // Loading spline
    std::map<std::string, Function> kickSplines;
    try {
        kickSplines = Function::fromFile(filename);
    } catch (const rhoban_utils::JsonParsingError & exc) {
        logger.log("%s", exc.what());
    }
    for (auto &entry : kickSplines) {
        double duration = entry.second.getXMax();
        if (duration > xMax) {
            xMax = duration;
        }
    }
    logger.log("Maximum time: %f", xMax);

    // Retrieving degrees of freedom
    auto &goal = getServices()->model->goalModel().get();
    for (auto dof : dofs) {
        lateralStepModel.setDOF(dof, goal.getDOF(dof));
    }

    // Retrieving feet positions at the begining
    auto leftPosition = lateralStepModel.position("left_foot_tip", "origin");
    auto rightPosition = lateralStepModel.position("right_foot_tip", "origin");
    auto leftOrientation = MatrixToEuler(lateralStepModel.orientation("left_foot_tip", "origin").transpose(), EulerYawPitchRoll);
    auto rightOrientation = MatrixToEuler(lateralStepModel.orientation("right_foot_tip", "origin").transpose(), EulerYawPitchRoll);

    // Generating 
    double T = 0;
    while (t < xMax) {
        T += dt;
        double remap = kickSplines["remap"].get(t);
        if (remap < 1e-6) remap = 1;
        t += dt*remap;

        // Updating the position
        inv.targetPosition("right_foot").x() = rightPosition.x()+kickSplines["shoot_x"].get(t);
        inv.targetPosition("right_foot").y() = rightPosition.y()+kickSplines["shoot_y"].get(t);
        inv.targetPosition("right_foot").z() = rightPosition.z()+kickSplines["shoot_z"].get(t);
        
        inv.targetPosition("left_foot").x() = leftPosition.x()+kickSplines["support_x"].get(t);
        inv.targetPosition("left_foot").y() = leftPosition.y()+kickSplines["support_y"].get(t);
        inv.targetPosition("left_foot").z() = leftPosition.z()+kickSplines["support_z"].get(t);

        Eigen::Vector3d shootTargetOrientation;
        Eigen::Vector3d supportTargetOrientation;

        shootTargetOrientation.x() = rightOrientation.x() + deg2rad(kickSplines["shoot_yaw"].get(t));
        shootTargetOrientation.y() = rightOrientation.y() + deg2rad(kickSplines["shoot_pitch"].get(t));
        shootTargetOrientation.z() = rightOrientation.z() + deg2rad(kickSplines["shoot_roll"].get(t));
        
        supportTargetOrientation.x() = leftOrientation.x() + deg2rad(kickSplines["support_yaw"].get(t));
        supportTargetOrientation.y() = leftOrientation.y() + deg2rad(kickSplines["support_pitch"].get(t));
        supportTargetOrientation.z() = leftOrientation.z() + deg2rad(kickSplines["support_roll"].get(t));

        inv.targetOrientation("right_foot") = EulerToMatrix(shootTargetOrientation, EulerYawPitchRoll);
        inv.targetOrientation("left_foot") = EulerToMatrix(supportTargetOrientation, EulerYawPitchRoll);

        if (lateralStepModel.getDOF("left_knee") < 0.01) lateralStepModel.setDOF("left_knee", 0.01);
        if (lateralStepModel.getDOF("right_knee") < 0.01) lateralStepModel.setDOF("right_knee", 0.01);
        inv.run(0.0001, 100);

        // Adding orientations target
        auto tmpModel = lateralStepModel;
        for (auto dof : dofs) {
            double delta = rad2deg(tmpModel.getDOF(dof)-goal.getDOF(dof));

            // Adding the extra angles
            auto name = dof;
            replaceAll(name, "right", "shoot");
            replaceAll(name, "left", "support");
            delta += kickSplines[name].get(t);
  
            splines[dof].addPoint(T, delta);
        }
    }

    logger.log("tMax = %f", T);
    tMax = T;
}

std::string LateralStep::cmdLateralStepGen()
{
    logger.log("Generating spline for lateral step");
    // Loading the kick
    loadLateralStep("lateral_step.json");
    // Writing it to output file
    logger.log("Generated spline, tMax=%f", tMax);
    Function::toFile(splines, "compiled_lateral_step.json");
    
    // Re loading kicks
    loadCompiledLateralSteps();

    return "Generated";
}

std::string LateralStep::getName()
{
    return "lateralStep";
}

void LateralStep::onStart()
{
    generated = false;
    bind->pull();
    
    if (live) {
      loadLateralStep("lateral_step.json");
    } 

    logger.log("Starting a lateral step");
    t = 0;
    over = false;
    applied = false;
    generated = true;
    Helpers::getServices()->model->setReadBaseUpdate(true);
}

void LateralStep::onStop(){
    Helpers::getServices()->model->setReadBaseUpdate(false);  
}

void LateralStep::apply()
{
  /*
    auto loc = getServices()->localisation;
    loc->applyKick(100 * distX, 100 * distY);
  */
}

void LateralStep::step(float elapsed)
{
    if (!generated) {
        return;
    }

    bind->pull();

    if (t > tMax && !applied) {
        applied = true;
        apply();
    }

    if (t > tMax ) {
        if (!over) {
            logger.log("Kick is over");
            over = true;
            this->Move::stop();
        }
    } else {
        t += elapsed;
        
        for (auto dof : dofs) {
            auto name = dof;
            double sign = 1;

            // Mirroring the play for the left foot
            if (left) {
                if (strContains(name, "left")) {
                    replaceAll(name, "left", "right");
                } else {
                    replaceAll(name, "right", "left");
                }

                if (strContains(name, "roll") || strContains(name, "yaw")) {
                    sign = -1;
                }
            }

            setAngle(dof, splines[name].get(t)*sign);
        }
    }

    bind->push();
}
