#include <iostream>
#include <stdexcept>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include <Utils/Angle.h>
#include <Model/NamesModel.h>
#include <rhoban_utils/timing/time_stamp.h>
#include "services/RobotModelService.h"
#include "services/LocalisationService.h"
#include "moves/Move.h"
#include "scheduler/MoveScheduler.h"
#include <string>
#include <vector>
#include <algorithm>

using namespace rhoban_utils;

/**
 * Log
 *  - DOF goals
 *  - DOF reads
 *  - Pressure
 *  - IMU
 *  - Camera pose
 */
RobotModelService::RobotModelService()
{
  goalModel.startServer();
}

bool RobotModelService::tick(double elapsed)
{
  RhAL::StandardManager* manager = getScheduler()->getManager();

  for (const std::string& name : goalModel.getDofNames())
  {
    goalModel.setDof(name, RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).goalPosition().getWrittenValue()));
  }

  goalModel.publishModel(false);

  return true;
}

void RobotModelService::flushAll(double gain)
{
  flush(true, true, true, true, true, gain);
}
void RobotModelService::flushArms(double gain)
{
  flush(false, true, true, false, false, gain);
}
void RobotModelService::flushHead(double gain)
{
  flush(true, false, false, false, false, gain);
}
void RobotModelService::flushLegs(double gain)
{
  flush(false, false, false, true, true, gain);
}
void RobotModelService::flushLeftLeg(double gain)
{
  flush(false, false, false, true, false, gain);
}
void RobotModelService::flushRightLeg(double gain)
{
  flush(false, false, false, false, true, gain);
}

void RobotModelService::flush(bool doHead, bool doLeftArm, bool doRightArm, bool doLeftLeg, bool doRightLeg,
                              double gain)
{
  if (doHead)
  {
    for (std::string dof : { "head_pitch", "head_yaw" })
    {
      setAngle(dof, rad2deg(goalModel.getDof(dof)) * gain);
    }
  }

  if (doLeftArm)
  {
    for (std::string dof : { "left_shoulder_pitch", "left_shoulder_roll", "left_elbow" })
    {
      setAngle(dof, rad2deg(goalModel.getDof(dof)) * gain);
    }
  }

  if (doRightArm)
  {
    for (std::string dof : { "right_shoulder_pitch", "right_shoulder_roll", "right_elbow" })
    {
      setAngle(dof, rad2deg(goalModel.getDof(dof)) * gain);
    }
  }

  if (doLeftLeg)
  {
    for (std::string dof :
         { "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll" })
    {
      setAngle(dof, rad2deg(goalModel.getDof(dof)) * gain);
    }
  }

  if (doRightLeg)
  {
    for (std::string dof : { "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch",
                             "right_ankle_roll" })
    {
      setAngle(dof, rad2deg(goalModel.getDof(dof)) * gain);
    }
  }
}