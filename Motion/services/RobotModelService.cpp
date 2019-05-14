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

  // Setting written values to the DOFs of the goal model
  for (const std::string& name : goalModel.getDofNames())
  {
    goalModel.setDof(name, RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).goalPosition().getWrittenValue()));
  }

  // Publishing goal model to ZMQ
  server.publishModel(goalModel, false);

  return true;
}