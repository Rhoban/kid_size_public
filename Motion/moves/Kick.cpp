#include <math.h>
#include <services/ModelService.h>
#include <services/LocalisationService.h>
#include <services/StrategyService.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <Utils/Euler.h>
#include <rhoban_utils/util.h>
#include "Kick.h"

static rhoban_utils::Logger logger("kick");
using namespace Leph;
using namespace rhoban_utils;

// DOFs
static std::vector<std::string> dofs = {
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll", "left_knee", "left_ankle_pitch", "left_ankle_roll"
};

Kick::Kick()
    : kickModel(Leph::SigmabanModel), 
    inv(kickModel)
{
    initializeBinding();

    bind->bindNew("over", over, RhIO::Bind::PushOnly)
        ->defaultValue(false)->comment("Is the kick over");
    
    bind->bindNew("live", live, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Live tuning?");
    
    bind->bindNew("left", left, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Left foot");
    
    bind->bindNew("kickName", kickName, RhIO::Bind::PullOnly)
        ->defaultValue("classic")->comment("Name of the kick which should be used");

    bind->bindNew("useManualT", useManualT, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Use manual time");

    bind->bindNew("manualT", manualT, RhIO::Bind::PullOnly)
        ->defaultValue(0)->comment("Manual T");

    bind->bindNew("dt", dt, RhIO::Bind::PullOnly)
        ->defaultValue(0.005)->comment("Dt for kick gen");

    bind->bindFunc("kickGen", "Pre-compute the kick",
        &Kick::cmdKickGen, *this);

    bind->bindNew("applyKickRatio", applyKickRatio, RhIO::Bind::PullOnly)
        ->defaultValue(0.95)->comment("Ratio in the kick timing when the kick should be applied")
        ->persisted(true);

    bind->bindNew("t", t, RhIO::Bind::PushOnly)
        ->defaultValue(0.0);

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

    // Load available kicks
    kmc.loadFile();
}
        
void Kick::set(bool left, const std::string & newKickName)
{
    bind->node().setBool("left", left);
    bind->node().setStr("kickName", newKickName);
}

std::map<std::string, Function> Kick::loadCompiledKick(std::string filename)
{
    std::map<std::string, Function> splines;
    try { 
      splines = Function::fromFile(filename);
    } catch (const rhoban_utils::JsonParsingError & exc) {
      logger.error("%s", exc.what());
      return splines;
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
        
void Kick::loadCompiledKicks()
{
    for (const std::string & kickName: kmc.getKickNames()) {
        std::string path = getCompiledPath(kickName);
        preloadedSplines[kickName] = loadCompiledKick(path);
        maxTimes[kickName] = tMax;

        // Trying to load the specific for left foot
        std::string left_path = getCompiledPath(kickName, true);
        if (file_exists(left_path)) {
            logger.log("Loading the left kick %s", left_path.c_str());
            preloadedSplines[kickName+"_left"] = loadCompiledKick(left_path);
            maxTimes[kickName+"_left"] = tMax;
        }
    }
}

void Kick::loadKick(std::string filename)
{
    bind->pull();
    splines.clear();
    double t = 0;
    double xMax = 0;

    logger.log("Generating kick");

    // Loading spline
    std::map<std::string, Function> kickSplines = Function::fromFile(filename);
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
        kickModel.setDOF(dof, goal.getDOF(dof));
    }

    // Retrieving feet positions at the begining
    auto leftPosition = kickModel.position("left_foot_tip", "origin");
    auto rightPosition = kickModel.position("right_foot_tip", "origin");
    auto leftOrientation = MatrixToEuler(kickModel.orientation("left_foot_tip", "origin").transpose(), EulerYawPitchRoll);
    auto rightOrientation = MatrixToEuler(kickModel.orientation("right_foot_tip", "origin").transpose(), EulerYawPitchRoll);

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

        if (kickModel.getDOF("left_knee") < 0.01) kickModel.setDOF("left_knee", 0.01);
        if (kickModel.getDOF("right_knee") < 0.01) kickModel.setDOF("right_knee", 0.01);
        inv.run(0.0001, 100);

        // Adding orientations target
        auto tmpModel = kickModel;
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

std::string Kick::getPath(const std::string kickName, bool left)
{
  if (left) {
    return "kick_" + kickName + "_left.json";
  } else {
    return "kick_" + kickName + ".json";
  }
}

std::string Kick::getCompiledPath(const std::string kickName, bool left)
{
  if (left) {
    return "compiled_kick_" + kickName + "_left.json";
  } else {
    return "compiled_kick_" + kickName + ".json";
  }
}

std::string Kick::cmdKickGen()
{
  try {
      for (const std::string & kickName : kmc.getKickNames()) {
          logger.log("Generating spline for '%s' kick", kickName.c_str());
          // Loading the kick
          loadKick(getPath(kickName));
          // Writing it to output file
          logger.log("Generated spline, tMax=%f", tMax);
          Function::toFile(splines, getCompiledPath(kickName));
      
          // Trying to compile specific splines for left foot
          if (file_exists(getPath(kickName, true))) {
              // Loading the kick
              loadKick(getPath(kickName, true));
              // Writing it to output file
              logger.log("Generated spline, specific for left, tMax=%f", tMax);
              Function::toFile(splines, getCompiledPath(kickName, true));
          }
      }

      // Re loading kicks
      loadCompiledKicks();
  } catch (const rhoban_utils::JsonParsingError & exc) {
      logger.error("%s", exc.what());
      return exc.what();
  }

  return "Generated";
}

std::string Kick::getName()
{
    return "kick";
}

void Kick::onStart()
{
    generated = false;
    bind->pull();

    if (live) {
        if (left && file_exists(getPath(kickName, true))) {
            loadKick(getPath(kickName, true));
        } else {
            loadKick(getPath(kickName));
        }
    } else {
        try{
            if (left) {
                // Trying to load specific splines
                auto tmp = kickName+"_left";
                if (preloadedSplines.count(tmp)) {
                    splines = preloadedSplines.at(tmp);
                    tMax = maxTimes.at(tmp);
                } else {
                    splines = preloadedSplines.at(kickName);
                    tMax = maxTimes.at(kickName);
                }
            } else {
                splines = preloadedSplines.at(kickName);
                tMax = maxTimes.at(kickName);
            }
        } catch (const std::out_of_range & exc) {
            logger.error("Kick::onStart: unknown kick '%s'", kickName.c_str());
        }
    }

    logger.log("Starting kick '%s', tMax=%f", kickName.c_str(), tMax);
    t = 0;
    over = false;
    applied = false;
    generated = true;

    // Announce that a kick has been performed (information is shared to other robots)
    getServices()->strategy->announceKick();    
}

void Kick::apply()
{
    const csa_mdp::KickModel & kick_model = kmc.getKickModel(kickName);

    Eigen::Vector2d initialBallPos(0,0), finalBallPos;

    finalBallPos = kick_model.getKickInSelf(initialBallPos, !left);

    double distX = finalBallPos(0);
    double distY = finalBallPos(1);
    logger.log("Applying kick x=%f, y=%f", distX, distY);
    
    auto loc = getServices()->localisation;
    loc->applyKick(100 * distX, 100 * distY);
}

void Kick::step(float elapsed)
{
    if (!generated) {
        return;
    }

    bind->pull();

    if (t > tMax*applyKickRatio && !applied) {
        applied = true;
        apply();
    }

    if (t > tMax && !useManualT) {
        if (!over) {
            logger.log("Kick is over");
            over = true;
            this->Move::stop();
        }
    } else {
        t += elapsed;

        if (useManualT) {
            t = manualT;
            if (t < 0) t = 0;
            if (t > tMax) t = tMax;
        }
        
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
