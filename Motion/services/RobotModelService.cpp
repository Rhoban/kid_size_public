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
}

bool RobotModelService::tick(double elapsed)
{
  RhAL::StandardManager* manager = getScheduler()->getManager();

  for (const std::string& name : goalModel.getDofNames())
  {
    // Setting written values to the DOFs of the goal model
    double goalValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).goalPosition().getWrittenValue());
    goalModel.setDof(name, goalValue);

    // Setting read values to the DOFs of the read model
    double readValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).position().readValue().value);
    readModel.setDof(name, Helpers::isFakeMode() ? goalValue : readValue);
  }

  // Using the
  readModel.setImu(true, getYaw(), getPitch(), getRoll());

  if (getPressureLeftRatio() > 0.9)
  {
    readModel.setSupportFoot(readModel.Left, true);
  }
  if (getPressureRightRatio() > 0.9)
  {
    readModel.setSupportFoot(readModel.Right, true);
  }

  // Publishing goal model to ZMQ
  server.publishModel(readModel, false);

  return true;
}