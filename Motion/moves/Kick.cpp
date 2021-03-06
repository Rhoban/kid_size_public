#include <math.h>
#include <services/LocalisationService.h>
#include <services/StrategyService.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>
#include "Kick.h"
#include "Head.h"
#include "Arms.h"
#include "Walk.h"
#include "services/DecisionService.h"
#include "services/ModelService.h"

#include <set>

static rhoban_utils::Logger logger("kick");
using namespace rhoban_utils;

// DOFs
static std::vector<std::string> dofs = { "right_hip_yaw",       "right_hip_pitch",      "right_hip_roll",
                                         "right_knee",          "right_ankle_pitch",    "right_ankle_roll",
                                         "right_shoulder_roll", "right_shoulder_pitch", "right_elbow",
                                         "left_hip_yaw",        "left_hip_pitch",       "left_hip_roll",
                                         "left_knee",           "left_ankle_pitch",     "left_ankle_roll",
                                         "left_shoulder_roll",  "left_shoulder_pitch",  "left_elbow" };

Kick::Kick(Head* _head, Walk* _walk, Arms* _arms) : head(_head), walk(_walk), arms(_arms)
{
  initializeBinding();

  bind->bindNew("over", over, RhIO::Bind::PushOnly)->defaultValue(false)->comment("Is the kick over");

  bind->bindNew("live", live, RhIO::Bind::PullOnly)->defaultValue(false)->comment("Live tuning?");

  bind->bindNew("left", left, RhIO::Bind::PullOnly)->defaultValue(false)->comment("Left foot");

  bind->bindNew("kickName", kickName, RhIO::Bind::PullOnly)
      ->defaultValue("classic")
      ->comment("Name of the kick which should be used");

  bind->bindNew("useManualT", useManualT, RhIO::Bind::PullOnly)->defaultValue(false)->comment("Use manual time");

  bind->bindNew("manualT", manualT, RhIO::Bind::PullOnly)->defaultValue(0)->comment("Manual T");

  bind->bindNew("dt", dt, RhIO::Bind::PullOnly)->defaultValue(0.005)->comment("Dt for kick gen");

  bind->bindFunc("kickGen", "Pre-compute the kick", &Kick::cmdKickGen, *this);

  bind->bindNew("t", t, RhIO::Bind::PushOnly)->defaultValue(0.0);

  // Cooldown and warmup
  bind->bindNew("cooldown", cooldown, RhIO::Bind::PullOnly)->defaultValue(0.5)->comment("Cooldown duration [s]");
  bind->bindNew("warmup", warmup, RhIO::Bind::PullOnly)->defaultValue(0.4)->comment("Warmup [s]");

  // Pause and pausetime
  bind->bindNew("pause", pause, RhIO::Bind::PullOnly)->defaultValue(false);

  // Is that kick cancellable ?
  bind->bindNew("cancellable", cancellable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("enableCancel", enableCancel, RhIO::Bind::PullOnly)->defaultValue(true);

  // Load available kicks
  kmc.loadFile();
}

void Kick::set(bool _left, const std::string& _kickName, bool _pause, bool _cancellable)
{
  bind->node().setBool("cancellable", _cancellable);
  bind->node().setBool("pause", _pause);
  bind->node().setBool("left", _left);
  bind->node().setStr("kickName", _kickName);
}

bool Kick::shouldCancel()
{
  // Cancel was forced
  if (forceCancel)
  {
    return true;
  }

  if (!enableCancel || !cancellable || kickName == "jump" || kickName == "throwin")
  {
    // This kick can't be cancelled at all
    return false;
  }

  if (t > tKick)
  {
    // Kick is already ongoing, it can't be cancelled now
    return false;
  }

  // Check if ball is in kick zone or not
  auto localisation = getServices()->localisation;
  auto ball = localisation->getBallPosSelf(true);
  const csa_mdp::KickZone& kick_zone = kmc.getKickModel(kickName).getKickZone();
  Eigen::Vector3d wishedPos = kick_zone.getWishedPos(!left);
  Eigen::Vector3d ballPos(ball.x, ball.y, wishedPos[2]);
  Eigen::Vector3d avg = 0.75 * wishedPos + 0.25 * ballPos;
  avg.z() = -avg.z();

  return !kick_zone.canKick(!left, avg);
}

void Kick::unpause()
{
  bind->node().setBool("pause", false);
}

void Kick::cancel()
{
  forceCancel = true;
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
  std::set<std::string> unsupportedEntries = { "shoot_x",     "shoot_y",     "shoot_z",       "shoot_yaw",
                                               "shoot_pitch", "shoot_roll",  "support_x",     "support_y",
                                               "support_z",   "support_yaw", "support_pitch", "support_roll" };

  // Loading spline
  std::map<std::string, Function> kickSplines = Function::fromFile(filename);
  for (auto& entry : kickSplines)
  {
    const std::string& entryName = entry.first;
    if (unsupportedEntries.count(entryName) > 0)
    {
      logger.warning("%s in file %s: '%s' is not supported anymore", DEBUG_INFO.c_str(), filename.c_str(),
                     entryName.c_str());
    }
    double duration = entry.second.getXMax();
    if (duration > xMax)
    {
      xMax = duration;
    }
  }
  logger.log("Maximum time: %f", xMax);

  // Generating
  double T = 0;
  tKick = -1;
  while (t < xMax)
  {
    T += dt;
    double remap = kickSplines["remap"].get(t);
    if (remap < 1e-6)
      remap = 1;
    t += dt * remap;

    if (kickSplines.count("kick") && kickSplines["kick"].get(t) > 0.5 && tKick < 0)
    {
      tKick = T;
    }

    for (auto dof : dofs)
    {
      auto name = dof;
      double sign = 1;
      replaceAll(name, "right", "shoot");
      replaceAll(name, "left", "support");
      /* If there is no shoot or support, try symetrical splines*/
      if (kickSplines.count(name) == 0)
      {
        replaceAll(name, "shoot_", "");
        replaceAll(name, "support_", "");

        if (strContains(name, "roll") && strContains(dof, "left"))
        {
          sign = -1;
        }
      }
      double val = 0;
      if (kickSplines.count(name) > 0)
      {
        val = sign * kickSplines[name].get(t);
      }

      splines[dof].addPoint(T, val);
    }
  }

  tMax = T;
  if (tKick < 0)
  {
    logger.warning("No tKick for %s, using t=0", filename.c_str());
    tKick = 0;
  }
  logger.log("tMax = %f, tKick=%f", tMax, tKick);
}

std::string Kick::getPath(const std::string kickName, bool left)
{
  if (left)
  {
    return "kick_" + kickName + "_left.json";
  }
  else
  {
    return "kick_" + kickName + ".json";
  }
}

std::string Kick::cmdKickGen()
{
  preloadedSplines.clear();
  maxTimes.clear();

  try
  {
    for (const std::string& kickName : kmc.getKickNames())
    {
      logger.log("Generating spline for '%s' kick", kickName.c_str());
      // Loading the kick
      loadKick(getPath(kickName));
      preloadedSplines[kickName] = splines;
      maxTimes[kickName] = tMax;
      kickTimes[kickName] = tKick;

      // Writing it to output file
      logger.log("Generated spline, tMax=%f", tMax);

      // Trying to compile specific splines for left foot
      if (file_exists(getPath(kickName, true)))
      {
        // Loading the kick
        loadKick(getPath(kickName, true));
        std::string tmp = kickName + "_left";
        preloadedSplines[tmp] = splines;
        maxTimes[tmp] = tMax;
        kickTimes[tmp] = tKick;
        // Writing it to output file
        logger.log("Generated spline, specific for left, tMax=%f", tMax);
      }
    }
  }
  catch (const rhoban_utils::JsonParsingError& exc)
  {
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
  forceCancel = false;
  bind->pull();

  if (kickName == "throwin")
  {
    headWasDisabled = head->isDisabled();
    head->setDisabled(true);
  }

  if (kickName == "throwin" || kickName == "jump")
  {
    arms->setArms(Arms::ArmsState::ArmsDisabled);
  }

  if (live)
  {
    if (left && file_exists(getPath(kickName, true)))
    {
      loadKick(getPath(kickName, true));
    }
    else
    {
      loadKick(getPath(kickName));
    }
  }
  else
  {
    try
    {
      if (left)
      {
        // Trying to load specific splines
        auto tmp = kickName + "_left";
        if (preloadedSplines.count(tmp))
        {
          splines = preloadedSplines.at(tmp);
          tMax = maxTimes.at(tmp);
          tKick = kickTimes.at(tmp);
        }
        else
        {
          splines = preloadedSplines.at(kickName);
          tMax = maxTimes.at(kickName);
          tKick = kickTimes.at(kickName);
        }
      }
      else
      {
        splines = preloadedSplines.at(kickName);
        tMax = maxTimes.at(kickName);
        tKick = kickTimes.at(kickName);
      }
    }
    catch (const std::out_of_range& exc)
    {
      logger.error("Kick::onStart: unknown kick '%s'", kickName.c_str());
    }
  }

  logger.log("Starting kick '%s', tMax=%f", kickName.c_str(), tMax);
  t = 0;
  over = false;
  applied = false;
  generated = true;

  kickState = KickWaitingWalkToStop;
}

void Kick::onStop()
{
  if (kickName == "throwin")
  {
    head->setDisabled(headWasDisabled);
  }
  if (kickName == "throwin" || kickName == "jump")
  {
    arms->setArms(Arms::ArmsState::ArmsEnabled);
  }
}

void Kick::apply()
{
  const csa_mdp::KickModel& kick_model = kmc.getKickModel(kickName);

  Eigen::Vector2d initialBallPos(0, 0), finalBallPos;

  finalBallPos = kick_model.getKickInSelf(initialBallPos, !left);

  double distX = finalBallPos(0);
  double distY = finalBallPos(1);
  logger.log("Applying kick x=%f, y=%f", distX, distY);

  auto loc = getServices()->localisation;
  loc->applyKick(distX, distY);
}

void Kick::step(float elapsed)
{
  if (!generated)
  {
    return;
  }

  bind->pull();

  // While kicking, walk is force disabled
  walk->control(false);

  if (kickState == KickWarmup || kickState == KickCooldown)
  {
    t += elapsed;
  }

  if (kickState == KickWaitingWalkToStop && !walk->isWalking())
  {
    // Forcing support foot in the model
    getServices()->model->model.setSupportFoot(left ? rhoban::HumanoidModel::Right : rhoban::HumanoidModel::Left, true);

    // Walk is over, go to warmup state
    kickState = KickWarmup;
    t = 0;
  }
  DecisionService* decision = getServices()->decision;

  if (kickState == KickWarmup && t >= warmup && !decision->freezeKick)
  {
    // Warmup over, start the kick move
    kickState = KickKicking;
    t = 0;
  }
  if (kickState == KickKicking)
  {
    // If we are in pause mode, block the kick on the pause time
    if (pause && t > tKick)
    {
      t = tKick;
    }

    if (shouldCancel())
    {
      logger.log("Cancelling kick");
      kickState = KickCancelling;
    }
    else
    {
      if (t > tKick && !applied)
      {
        applied = true;
        apply();
        getServices()->strategy->announceKick();
      }

      // Reading the actual kick spline
      stepSpline(elapsed);

      if (over)
      {
        // Kick is over, enter the coolDown sequence
        kickState = KickCooldown;
        t = 0;
      }
    }
  }

  if (kickState == KickCancelling)
  {
    stepSpline(-elapsed);

    if (t < 0)
    {
      kickState = KickCooldown;
    }
  }
  if (kickState == KickCooldown && t >= cooldown)
  {
    logger.log("Cooldown finished (%f > %f)", t, cooldown);
    // Cooldown is over, stopping the kick
    this->Move::stop();
  }
}

void Kick::stepSpline(float elapsed)
{
  if (t > tMax && !useManualT)
  {
    if (!over)
    {
      logger.log("Kick is over");
      over = true;
    }
  }
  else
  {
    t += elapsed;

    if (useManualT)
    {
      t = manualT;
      if (t < 0)
        t = 0;
      if (t > tMax)
        t = tMax;
    }

    for (auto dof : dofs)
    {
      auto name = dof;
      double sign = 1;

      // Mirroring the play for the left foot
      if (left && kickName != "throwin")
      {
        if (strContains(name, "left"))
        {
          replaceAll(name, "left", "right");
        }
        else
        {
          replaceAll(name, "right", "left");
        }

        if (strContains(name, "roll") || strContains(name, "yaw"))
        {
          sign = -1;
        }
      }

      setAngle(dof, splines[name].get(t) * sign);
    }
  }

  bind->push();
}
