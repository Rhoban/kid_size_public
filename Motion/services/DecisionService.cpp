#include "DecisionService.h"
#include "LocalisationService.h"
#include "TeamPlayService.h"
#include "RefereeService.h"
#include "StrategyService.h"
#include "rhoban_utils/logging/logger.h"
#include "robocup_referee/constants.h"
#include "moves/Head.h"
#include "moves/Move.h"
#include "moves/Walk.h"
#include <hl_communication/wrapper.pb.h>

using namespace hl_communication;
using namespace hl_monitoring;
using namespace rhoban_utils;
using namespace rhoban_team_play;
using namespace robocup_referee;

static rhoban_utils::Logger logger("Decision");

DecisionService::DecisionService()
  : isBallMoving(false), camera_status(FrameStatus::UNKNOWN_FRAME_STATUS), bind("decision")
{
  // Ball quality
  bind.bindNew("ballQThreshold", ballQThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.7)
      ->comment("Threshold to enable the ball good")
      ->persisted(true);
  bind.bindNew("ballQDisableThreshold", ballQDisableThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.3)
      ->comment("Threshold to disable the ball good")
      ->persisted(true);
  bind.bindNew("isBallQualityGood", isBallQualityGood)->comment("Is ball quality good ?")->defaultValue(false);
  bind.bindNew("isBallMoving", isBallMoving, RhIO::Bind::PushOnly)
      ->comment("Is the ball moving significantly according to one of the robot of the team")
      ->defaultValue(false);
  bind.bindNew("hasMateKickedRecently", hasMateKickedRecently, RhIO::Bind::PushOnly)
      ->comment("True if one of the robot of the team has performed a kick recently")
      ->defaultValue(false);
  bind.bindNew("isMateKicking", isMateKicking, RhIO::Bind::PushOnly)
      ->comment("True if one of the robot of the team is performing a kick")
      ->defaultValue(false);

  // Constraint to say that ball is moving
  bind.bindNew("movingBallMinSpeed", movingBallMinSpeed, RhIO::Bind::PullOnly)
      ->comment("Ball is considered to move if it has a speed higher than this value [m/s]")
      ->defaultValue(0.5);
  bind.bindNew("postKickTrackingTime", postKickTrackingTime, RhIO::Bind::PullOnly)
      ->comment("Time during which a ball is considered as moving after a "
                "robot started performing a kick [s]")
      ->defaultValue(5);

  // Field quality
  bind.bindNew("fieldQThreshold", fieldQThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.7)
      ->comment("Threshold to enable the ball good")
      ->persisted(true);
  bind.bindNew("fieldQDisableThreshold", fieldQDisableThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.3)
      ->comment("Threshold to disable the ball good")
      ->persisted(true);
  bind.bindNew("isFieldQualityGood", isFieldQualityGood)->comment("Is field quality good ?")->defaultValue(false);

  // Robot fallen
  bind.bindNew("isFallen", isFallen)->comment("Is the robot fallen ?")->defaultValue(false);
  bind.bindNew("timeSinceFall", timeSinceFall, RhIO::Bind::PushOnly)
      ->comment("Time elapsed since last fall [s]")
      ->defaultValue(0);
// TODO: solve issue with RhIO and enums
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
  bind.bindNew("fallDirection", (int&)fallDirection)
      ->comment("Direction of the robot fall ?")
      ->defaultValue(FallDirection::None);
  bind.bindNew("fallStatus", (int&)fallStatus)->comment("Status of the fall ")->defaultValue(FallStatus::Ok);
#pragma GCC diagnostic pop

  // Let play
  bind.bindNew("shouldLetPlay", shouldLetPlay, RhIO::Bind::PushOnly)->comment("Should let play?")->defaultValue(false);

  // Let play radius
  bind.bindNew("letPlayRadius", letPlayRadius, RhIO::Bind::PushOnly)->comment("Let play radius [m]");

  // Shared infos
  bind.bindNew("enableShare", enableShare, RhIO::Bind::PullOnly)->defaultValue(true)->persisted(true);
  bind.bindNew("shareFieldQ", shareFieldQ, RhIO::Bind::PullOnly)
      ->comment("Required field Q for shared")
      ->defaultValue(0.8)
      ->persisted(true);
  bind.bindNew("shareBallQ", shareBallQ, RhIO::Bind::PullOnly)
      ->comment("Required ball Q for shared")
      ->defaultValue(0.8)
      ->persisted(true);
  bind.bindNew("shareId", shareId, RhIO::Bind::PushOnly)->comment("The robot that shared the ball with us");
  bind.bindNew("shareSmooth", shareSmooth, RhIO::Bind::PullOnly)
      ->comment("Share smoothing")
      ->defaultValue(0.99)
      ->persisted(true);
  bind.bindNew("fakeTeamDecisions", fakeTeamDecisions, RhIO::Bind::PullOnly)->defaultValue(false)->persisted(true);

  // Shared ball position
  bind.bindNew("shareX", shareX)->defaultValue(4.5);
  bind.bindNew("shareY", shareY)->defaultValue(3);

  // Is the ball shared?
  bind.bindNew("ballIsShared", ballIsShared)->defaultValue(false);

  // Pressure and handling
  bind.bindNew("lowPressureThreshold", lowPressureThreshold, RhIO::Bind::PullOnly)
      ->comment("Low pressure to detect robot handling")
      ->defaultValue(60000)
      ->persisted(true);

  bind.bindNew("handled", handled)->comment("Is the robot handled?")->defaultValue(false);

  bind.bindNew("handledDelay", handledDelay)->comment("Time before robot goes to `handled` state")->defaultValue(0.5);

  // Freeze the kick
  bind.bindNew("freezeKick", freezeKick, RhIO::Bind::PushOnly)->comment("Freezing the kick")->defaultValue(false);

  // Cooperation
  bind.bindNew("cooperation", cooperation, RhIO::Bind::PullOnly)
      ->comment("Enabling the cooperation with team")
      ->defaultValue(true)
      ->persisted(true);

  // Who is the goal ?
  bind.bindNew("goalId", goalId, RhIO::Bind::PullOnly)->comment("Id of the goal")->defaultValue(2);

  bind.bindNew("nextKickIsThrowIn", nextKickIsThrowIn, RhIO::Bind::PushOnly)
      ->comment("Is next kick a throw in ?")
      ->defaultValue(false);

  bind.bindNew("isKickRunning", isKickRunning, RhIO::Bind::PushOnly)
      ->comment("Is a kick running ?")
      ->defaultValue(false);

  bind.bindNew("isThrowInRunning", isThrowInRunning, RhIO::Bind::PushOnly)
      ->comment("Is a throw in running ?")
      ->defaultValue(false);

  bind.bindNew("throwInEnable", throwInEnable, RhIO::Bind::PullOnly)
      ->comment("Is throw in enabled ?")
      ->defaultValue(true);

  selfAttackingT = 0;
  handledT = 0;

  // Ensuring all default values have been written
  bind.pull();
}

bool DecisionService::tick(double elapsed)
{
  auto loc = getServices()->localisation;
  auto teamPlay = getServices()->teamPlay;
  auto referee = getServices()->referee;
  auto strategy = getServices()->strategy;

  // Should we let the other players play?
  auto ballPos = loc->getBallPosSelf();

  if (isBallQualityGood)
  {
    lastSeenBallRight = (ballPos.y > 0);
  }

  // Computing the radius
  letPlayRadius = 0;

  shouldLetPlay = false;
  if (referee->isOpponentKickOffStart())
  {
    letPlayRadius = std::max(letPlayRadius, teamPlay->kickOffClearanceDist);
    shouldLetPlay = true;
  }

  Walk* walk = dynamic_cast<Walk*>(getMoves()->getMove("walk"));
  Head* head = dynamic_cast<Head*>(getMoves()->getMove("head"));
  if (walk->isRunning() && walk->isWalking())
  {
    camera_status = FrameStatus::SHAKING;
  }
  else if (head->isRunning() && !head->isDisabled())
  {
    camera_status = FrameStatus::MOVING;
  }
  else
  {
    camera_status = FrameStatus::STATIC;
  }

  freezeKick = false;

  if (referee->isGameInterruption())
  {
    if (referee->myTeamGameInterruption())
    {
      freezeKick = true;
      if (referee->isThrowIn() && throwInEnable)
      {
        nextKickIsThrowIn = true;
      }
    }
    else
    {
      letPlayRadius = std::max(letPlayRadius, teamPlay->gameInterruptionClearanceDist);
      shouldLetPlay = true;
    }
  }
  else if (referee->isRecentGameInterruption() && !referee->myTeamGameInterruption())
  {
    shouldLetPlay = true;
    letPlayRadius = std::max(letPlayRadius, teamPlay->gameInterruptionClearanceDist);
  }
  else if (!referee->isRecentGameInterruption())
  {
    nextKickIsThrowIn = false;
  }

  bind.pull();
  if (Helpers::isFakeMode())
  {
    bind.push();
    if (fakeTeamDecisions)
    {
      return true;
    }
  }

  // Shared ball
  bool ballWasShared = ballIsShared;
  ballIsShared = false;
  float bestDist = -1;

  // XXX Captain: this should be removed once captain is implemented
  //              This is quite outdated anyway
  //
  // Ball sharing is enabled, we are well localized on the field
  if (!isFallen && enableShare && isFieldQualityGood)
  {
    auto infos = teamPlay->allInfo();
    for (auto& entry : infos)
    {
      auto info = entry.second;
      int info_id = info.robot_id().robot_id();
      PerceptionExtra extra = extractPerceptionExtra(info.perception());
      // This is another player
      if (info_id != teamPlay->myId() && !isOutdated(info) && !referee->isPenalized(info_id))
      {
        // Its ball quality is good and our its quality is good
        if (extra.field().valid() && extra.ball().valid() && info.intention().action_planned() != Action::INACTIVE)
        {
          const PoseDistribution& pose = info.perception().self_in_field(0).pose();
          const PositionDistribution ball_in_self = info.perception().ball_in_self();
          // This player sees the ball and is well localized
          float dist = sqrt(pow(ball_in_self.x(), 2) + pow(ball_in_self.y(), 2));

          if (dist < 3 && (bestDist < 0 || dist < bestDist))
          {
            // We use the shared ball that is known to be nearest from
            // the broadcaster robot
            bestDist = dist;
            shareId = info_id;
            float a = pose.dir().mean();
            double ballX = ball_in_self.x();
            double ballY = ball_in_self.y();
            double fieldX = pose.position().x();
            double fieldY = pose.position().y();
            float X = fieldX + cos(a) * ballX - sin(a) * ballY;
            float Y = fieldY + sin(a) * ballX + cos(a) * ballY;

            const PositionDistribution& kickTarget = info.intention().kick().target();
            ballTargetX = kickTarget.x();
            ballTargetY = kickTarget.y();

            if (ballWasShared)
            {
              // Updating the ball position
              shareX = X * shareSmooth + shareX * (1 - shareSmooth);
              shareY = Y * shareSmooth + shareY * (1 - shareSmooth);
            }
            else
            {
              // Setting the ball position
              shareX = X;
              shareY = Y;
            }
            ballIsShared = true;
          }
        }
      }
    }
  }

  if (!Helpers::isFakeMode())
  {
    // Ball quality
    if (!isBallQualityGood && loc->ballQ > ballQThreshold)
    {
      isBallQualityGood = true;
    }
    if (isBallQualityGood && loc->ballQ < ballQDisableThreshold)
    {
      isBallQualityGood = false;
    }

    // Field quality
    if (!isFieldQualityGood && loc->fieldQ > fieldQThreshold)
    {
      isFieldQualityGood = true;
    }
    if (isFieldQualityGood && loc->fieldQ < fieldQDisableThreshold)
    {
      isFieldQualityGood = false;
    }
  }

  isKickRunning = getMoves()->getMove("kick")->isRunning();
  isThrowInRunning = isKickRunning && referee->isThrowIn();

  if (!Helpers::isFakeMode() || Helpers::isPython)
  {
    // Is the robot fallen?
    double pitch_angle = rad2deg(getPitch());
    double roll_angle = rad2deg(getRoll());

    double max_imu_angle = std::max(fabs(pitch_angle), fabs(roll_angle));

    double threshold_falling = 45;
    double threshold_fallen = 60;

    isFallen = !isKickRunning && max_imu_angle > threshold_falling;

    if (max_imu_angle < threshold_falling || isKickRunning)
    {
      fallStatus = FallStatus::Ok;
      fallDirection = FallDirection::None;
    }
    else
    {
      // Computing fallStatus
      if (max_imu_angle < threshold_fallen)
      {
        fallStatus = FallStatus::Falling;
      }
      else
      {
        fallStatus = FallStatus::Fallen;
      }
      // Computing fallDirection
      if (fabs(pitch_angle) > threshold_falling)
      {
        if (pitch_angle > 0)
        {
          fallDirection = FallDirection::Forward;
        }
        else
        {
          fallDirection = FallDirection::Backward;
        }
      }
      else
      {
        fallDirection = FallDirection::Side;
      }
    }

    bind.push();
  }

  if (!Helpers::isFakeMode() || Helpers::isPython)
  {
    // Detecting robot handling
    if (!isFallen && getPressureWeight() < lowPressureThreshold)
    {
      handledT += elapsed;
      if (handledT > handledDelay)
      {
        handled = true;
      }
    }
    else
    {
      handled = false;
      handledT = 0;
    }
  }

  if (isFallen)
  {
    timeSinceFall = 0;
  }
  else
  {
    timeSinceFall += elapsed;
  }

  // Update the isBallMoving and hasMateKickedRecently flags
  // a tmp value is used to ensure thread-safety
  const auto& teamplayInfos = getServices()->teamPlay->allInfo();
  bool tmpIsBallMoving = false;
  bool tmpHasMateKickedRecently = false;
  bool tmpIsMateKicking = false;
  if (strategy->getTimeSinceLastKick() < postKickTrackingTime)
  {
    tmpHasMateKickedRecently = true;
  }

  for (const auto& pair : teamplayInfos)
  {
    const RobotMsg& robotInfo = pair.second;
    MiscExtra misc_extra = extractMiscExtra(robotInfo);
    const PositionDistribution& ball_velocity = robotInfo.perception().ball_velocity_in_self();
    float vx = ball_velocity.x();
    float vy = ball_velocity.y();
    float ballSpeed = std::sqrt(vx * vx + vy * vy);
    if (misc_extra.time_since_last_kick() < postKickTrackingTime)
    {
      tmpHasMateKickedRecently = true;
    }

    // XXX: We only considering the ball as moving if WE are seeing it moving
    if (ballSpeed > movingBallMinSpeed && robotInfo.has_robot_id() &&
        robotInfo.robot_id().robot_id() == getServices()->teamPlay->myId())
    {
      tmpIsBallMoving = true;
    }

    if (robotInfo.intention().action_planned() == Action::KICKING)
    {
      tmpIsMateKicking = true;
    }
  }
  isBallMoving = tmpIsBallMoving;
  hasMateKickedRecently = tmpHasMateKickedRecently;
  isMateKicking = tmpIsMateKicking;

  bind.push();

  return true;
}
