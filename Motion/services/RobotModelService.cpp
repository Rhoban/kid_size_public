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
RobotModelService::RobotModelService() : odometryEnabled(false), timeSinceLastPublish(0)
{
}

bool RobotModelService::tick(double elapsed)
{
  RhAL::StandardManager* manager = getScheduler()->getManager();

  for (const std::string& name : model.getDofNames())
  {
    if (Helpers::isFakeMode())
    {
      // Setting written values to the DOFs of the goal model
      double goalValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).goalPosition().getWrittenValue());
      model.setDof(name, goalValue);
    }
    else
    {
      // Setting read values to the DOFs of the read model
      double readValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).position().readValue().value);
      model.setDof(name, readValue);
    }
  }

  if (!Helpers::isFakeMode() || Helpers::isPython)
  {
    // Using the IMU
    model.setImu(true, getYaw(), getPitch(), getRoll());

    // Integrating odometry
    if (odometryEnabled)
    {
      if (getPressureLeftRatio() > 0.9)
      {
        model.setSupportFoot(model.Left, true);
      }
      if (getPressureRightRatio() > 0.9)
      {
        model.setSupportFoot(model.Right, true);
      }
    }
  }

  // Publishing goal model to ZMQ
  timeSinceLastPublish += elapsed;
  if (timeSinceLastPublish > 0.01)
  {
    timeSinceLastPublish = 0;
    server.publishModel(model, false);
  }

  return true;
}

void RobotModelService::enableOdometry(bool enabled)
{
  odometryEnabled = enabled;
}