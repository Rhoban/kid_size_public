#include <math.h>
#include <services/LocalisationService.h>
#include <services/StrategyService.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>
#include "Kick.h"

#include <set>

static rhoban_utils::Logger logger("kick");
using namespace rhoban_utils;

// DOFs
static std::vector<std::string> dofs = {
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll", "left_knee", "left_ankle_pitch", "left_ankle_roll"
};

Kick::Kick()
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

    /// Previously, kick could use specific orders based on IK, this has been
    /// discontinued because the gap between theoric motion and real motion is
    /// too high
    std::set<std::string> unsupportedEntries = {
      "shoot_x", "shoot_y", "shoot_z", "shoot_yaw", "shoot_pitch", "shoot_roll", 
      "support_x", "support_y", "support_z","support_yaw", "support_pitch", "support_roll"
    };

    // Loading spline
    std::map<std::string, Function> kickSplines = Function::fromFile(filename);
    for (auto &entry : kickSplines) {
      const std::string & entryName = entry.first;
      if (unsupportedEntries.count(entryName) > 0) {
        logger.warning("%s in file %s: '%s' is not supported anymore",
                       DEBUG_INFO.c_str(), filename.c_str(), entryName.c_str());
      }
        double duration = entry.second.getXMax();
        if (duration > xMax) {
            xMax = duration;
        }
    }
    logger.log("Maximum time: %f", xMax);

    // Generating 
    double T = 0;
    while (t < xMax) {
        T += dt;
        double remap = kickSplines["remap"].get(t);
        if (remap < 1e-6) remap = 1;
        t += dt*remap;

        for (auto dof : dofs) {
          auto name = dof;
          replaceAll(name, "right", "shoot");
          replaceAll(name, "left", "support");
          splines[dof].addPoint(T, kickSplines[name].get(t));
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
