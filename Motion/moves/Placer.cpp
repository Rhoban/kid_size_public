#include <cmath>
#include <services/LocalisationService.h>
#include <services/TeamPlayService.h>
#include <rhoban_graphs/obstacle_avoider.h>
#include <robocup_referee/constants.h>
#include "Placer.h"
#include "Walk.h"

#include "rhoban_utils/logging/logger.h"

static rhoban_utils::Logger logger("Placer");

using namespace robocup_referee;
using namespace rhoban_utils;
using namespace rhoban_geometry;

Placer::Placer(Walk *walk)
  : walk(walk)
{
    initializeBinding();
    // Anti-SigFPE
    bind->pull();
    
    bind->bindNew("path", pathJson, RhIO::Bind::PushOnly);
    bind->bindNew("pathTargetDist", pathTargetDist, RhIO::Bind::PullOnly)
        ->defaultValue(0.7);
    bind->bindNew("tmpX", tmpX, RhIO::Bind::PushOnly);
    bind->bindNew("tmpY", tmpY, RhIO::Bind::PushOnly);

    bind->bindNew("targetX", targetX, RhIO::Bind::PullOnly)
        ->comment("[m]")
        ->defaultValue(0.0);
    bind->bindNew("errorX", errorX, RhIO::Bind::PushOnly)
        ->comment("[m]")
        ->defaultValue(0.0);
    bind->bindNew("targetY", targetY, RhIO::Bind::PullOnly)
        ->comment("[m]")
        ->defaultValue(0.0);
    bind->bindNew("errorY", errorY, RhIO::Bind::PushOnly)
        ->comment("[m]")
        ->defaultValue(0.0);
    bind->bindNew("targetAzimuth", targetAzimuth, RhIO::Bind::PullOnly);
    bind->bindNew("errorAzimuth", errorAzimuth, RhIO::Bind::PushOnly);

    bind->bindNew("stepP", stepper.k_p, RhIO::Bind::PullOnly)->defaultValue(1.0);
    bind->bindNew("lateralP", lateraler.k_p, RhIO::Bind::PullOnly)->defaultValue(1.0);
    bind->bindNew("turnP", turner.k_p, RhIO::Bind::PullOnly)->defaultValue(0.25);

    bind->bindNew("marginX", marginX, RhIO::Bind::PullOnly)->defaultValue(0.5)
        ->comment("[m]");
    bind->bindNew("marginY", marginY, RhIO::Bind::PullOnly)->defaultValue(0.5)
        ->comment("[m]");
    bind->bindNew("marginAzimuth", marginAzimuth, RhIO::Bind::PullOnly)
      ->comment("[deg]")->defaultValue(20);
    bind->bindNew("arrived", arrived, RhIO::Bind::PushOnly);
    bind->bindNew("hysteresis", hysteresis, RhIO::Bind::PullOnly)->defaultValue(1.5);

    bind->bindNew("dontWalk", dontWalk, RhIO::Bind::PullOnly);

    //DirectMode vars:
    bind->bindNew("lateralMode", lateralMode, RhIO::Bind::PullOnly)
      ->defaultValue(false)
      ->comment("using lateral step for lateral move only (with x drift control)");
    bind->bindNew("lmDriftXMax", lmDriftXMax, RhIO::Bind::PullOnly)
      ->defaultValue(0.2)
      ->comment("in lateral step mode, maximum X drift allowed before correction [m]");
    bind->bindNew("lmDriftXMaxHyst", lmDriftXMaxHyst, RhIO::Bind::PullOnly)
      ->defaultValue(0.1)
      ->comment("in lateral step mode, maximum X drift allowed before correction / Hysteresis [m]");
    
    bind->bindNew("directMode", directMode, RhIO::Bind::PullOnly)
        ->defaultValue(true)->comment("Turning to target before walking target");
    bind->bindNew("rushDistOk", rushDistOk, RhIO::Bind::PushOnly)
        ->defaultValue(false)->comment("Is the distance to the target suggesting to rush");
    bind->bindNew("rushAngleOk", rushAngleOk, RhIO::Bind::PushOnly)
        ->defaultValue(false)->comment("Is the angle toward the target valid for rush");
    bind->bindNew("rushHysteresisLow", rushHysteresisLow, RhIO::Bind::PullOnly)
        ->defaultValue(20)
        ->comment("Rush starts being allowed under this threshold [deg]");
    bind->bindNew("rushHysteresisHigh", rushHysteresisHigh, RhIO::Bind::PullOnly)
        ->defaultValue(30)
        ->comment("Rush stop being allowed above this threshold [deg]");
    bind->bindNew("capToTarget", capToTarget, RhIO::Bind::PushOnly)
        ->defaultValue(0)->comment("The direction of the target in the robot referential");
    bind->bindNew("rushRadiusHigh", rushRadiusHigh, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)
      ->comment("Above this threshold, rushDistOk becomes true [m]");
    bind->bindNew("rushRadiusLow", rushRadiusLow, RhIO::Bind::PullOnly)
        ->defaultValue(0.6)
        ->comment("Under this threshold, rushDistOk becomes false [m]");
    
    bind->bindNew("avoidOpponents", avoidOpponents, RhIO::Bind::PullOnly)
        ->defaultValue(true)->comment("Avoid the opponents?");
    bind->bindNew("avoidMates", avoidMates, RhIO::Bind::PullOnly)
        ->defaultValue(true)->comment("Avoid the team mates based on localisation?");
    bind->bindNew("avoidSharedOpponents", avoidSharedOpponents, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Avoid the shared opponents?");
    bind->pull();
}

void Placer::setLateralMode(bool l){
  bind->node().setBool("lateralMode",l);
}

void Placer::setDirectMode(bool l){
  bind->node().setBool("directMode",l);
}

float Placer::getMaxMarginXY(){
  return (marginX>marginY)?marginX:marginY;
}
/*
void Placer::setTemporaryMarginAzimuth(float l){
  bind->pull();
  tmpMarginAzimuth = marginAzimuth;
  bind->node().setBool("marginAzimuth",l);
  bind->push();  
}

void Placer::restoreMarginAzimuth(){
  bind->node().setBool("marginAzimuth",tmpMarginAzimuth);  
}
*/

void Placer::goTo(float x, float y, float azimuth, 
                  std::vector<Circle> obstacles_)
{
  //logger.log("goTo: %f (%f/%f) , %f (%f/%f) %f (%f/%f)",x, errorX,marginX,y,errorY,marginY,azimuth,errorAzimuth,marginAzimuth);
    bind->node().setFloat("targetX", x);
    bind->node().setFloat("targetY", y);
    bind->node().setFloat("targetAzimuth", azimuth);
    obstacles = obstacles_;
}

std::string Placer::getName()
{
    return "placer";
}

void Placer::onStart()
{
    auto teamPlay = getServices()->teamPlay;
    auto &info = teamPlay->selfInfo();
    info.placing = true;

    targetX = 0;
    targetY = 0;
    targetAzimuth = 0;
    // Reinitialize hysteresis when starting placer
    arrived = false;
    rushDistOk = false;
    rushAngleOk = false;
    logger.log("Starting");
}

void Placer::onStop()
{
    auto teamPlay = getServices()->teamPlay;
    auto &info = teamPlay->selfInfo();
    info.placing = false;
    
    setLateralMode(false);
    
    walk->control(false);
}

void Placer::step(float elapsed)
{
    (void) elapsed;// Elapsed is not used

    bind->pull();

    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();//[m]
    Angle cap(rad2deg(loc->getFieldOrientation()));

    // Target point on the field
    Point target(targetX, targetY);

    // Computing error in the robot basis
    Point error = (target - pos).rotation(-cap);
    errorX = error.x;
    errorY = error.y;

    // Azimuth to face the target
    capToTarget = Angle::fromXY(error.x, error.y).getSignedValue();

    // Error azimuth to the target azimuth
    errorAzimuth = (Angle(targetAzimuth) - cap).getSignedValue();

    bool goodPosition = fabs(errorX) < marginX && fabs(errorY) < marginY;
    bool goodPositionRestart = fabs(errorX) < marginX*hysteresis && fabs(errorY) < marginY*hysteresis;
    bool goodAlignment = fabs(errorAzimuth) < marginAzimuth;
    bool goodAlignmentRestart = fabs(errorAzimuth) < marginAzimuth*hysteresis;

    // Has the robot arrived to destination?
    if (arrived) {
        if (!goodPositionRestart || !goodAlignmentRestart) {
            logger.log("I am too far from destination, I'm on my way: errorX: %f, errorY: %f",
                    errorX, errorY);
            arrived = false;
        }
    } else {
        if (goodPosition && goodAlignment) {
            logger.log("I consider that I reached destination: errorX: %f, errorY: %f, errorAzimuth %f",
                    errorX, errorY, errorAzimuth);
            arrived = true;
        }
    }

    if (arrived) dontWalk = true;

    // Updating stepper properties (maxValues are in mm for walk)
    stepper.min = -walk->maxStepBackward / 1000;
    stepper.max = walk->maxStep / 1000;
    lateraler.min = -walk->maxLateral / 1000;
    lateraler.max = walk->maxLateral / 1000;
    turner.min = -walk->maxRotation;
    turner.max = walk->maxRotation;

    // The vector that we should follow
    double vecX = errorX;
    double vecY = errorY;
    double vecAzimuth = errorAzimuth;
    double vecCap = capToTarget;
    double vecDist = error.getLength();
    std::vector<Point> path;

    // No avoidance, the path is directly from here to the target
    path.push_back(pos);
    path.push_back(target);

    // Obstacle avoidance
    double score;
    rhoban_graphs::ObstacleAvoider avoider;

    // Obstacle in the robot frame
    if (obstacles.size()) {
        for (auto &obstacle : obstacles) {
            avoider.addObstacle(obstacle.getCenter(), obstacle.getRadius());
        }
    }

    // Adding nearest opponetn
    if (avoidOpponents) {
        for (const auto &opponent : loc->getOpponentsField()) {
            avoider.addObstacle(opponent, loc->opponentsRadius);
        }
    }

    if (avoidMates) {
      for (const auto & mate_entry : loc->getTeamMatesField()) {
        rhoban_geometry::Point pos(mate_entry.second(0),mate_entry.second(1));
        avoider.addObstacle(pos, loc->teamMatesRadius);
      }
    }

    if (avoidSharedOpponents) {
      //TODO: before using this option, a mechanism should get rid of nearby
      //shared opponents because there is a high risk that teammates see us as
      //an obstacle
      for (const Eigen::Vector2d & opp :loc->getSharedOpponents()) {
        avoider.addObstacle(Point(opp(0), opp(1)), loc->opponentsRadius);
      }
    }

    // Finding a path using the avoider
    auto result  = avoider.findPath(pos, target, 0.1, &score, [](Point pt) {
            return fabs(pt.x) < (Constants::field.fieldLength/2 + 0.5) &&
            fabs(pt.y) < (Constants::field.fieldWidth/2 + 0.5);
            });

    // The result is valid, using it instead of direct path
    if (result.size() > 1 && score > 0.1) {
        path = result;
    }

    // Building JSON path for RhIO (can be disabled in production)
    std::stringstream ss;
    ss << "[";
    for (auto pt : path) {
        ss << "[" << pt.x << "," << pt.y << "], ";
    }
    ss << "]";
    pathJson = ss.str();


    // Finding the next target point
    size_t k = 1;
    Point pathTarget, pathTargetRobot;
    int last = path.size()-1;
    pathTarget = path[last];

    for (;k < path.size(); k++) {
        auto A = path[k-1];
        auto B = path[k];

        double Adist = (A - pos).getLength();
        double Bdist = (B - pos).getLength();

        if (Bdist > pathTargetDist) {
            double delta = Bdist - Adist;
            double missing = pathTargetDist - Adist;
            double ratio = missing / delta;
            pathTarget = A + (B-A)*ratio;
            break;
        }
    }
    pathTargetRobot = (pathTarget - pos).rotation(-cap);

    tmpX = pathTarget.x;
    tmpY = pathTarget.y;
    vecX = pathTargetRobot.x;
    vecY = pathTargetRobot.y;
    vecCap = rad2deg(atan2(vecY, vecX));
    if (vecCap > 180) vecCap -= 360;
    // logger.log("vX: %f, vY: %f, vC: %f", vecX, vecY, vecCap);

    if (directMode) {
        if (!rushAngleOk && fabs(vecCap) < rushHysteresisLow) {
            rushAngleOk = true;
        }
        if (rushAngleOk && fabs(vecCap) > rushHysteresisHigh) {
            rushAngleOk = false;
        }
        if (!rushDistOk && vecDist > rushRadiusHigh) {
            rushDistOk = true;
        }
        if (rushDistOk && vecDist < rushRadiusLow) {
            rushDistOk = false;
        }
    }
    

    // While the robot has not arrived, update the orders
    if (!arrived) {
        //TODO maybe add another condition 'similar' to:  estimatedTime(rot+move+rot) < estimatedTime(move)
        if (directMode) {
            //logger.log("Direct mode is enabled");
            if (!rushDistOk) { // Use classic mode for short range
                stepper.update(vecX);
                lateraler.update(vecY);
                turner.update(vecAzimuth);
            } else if (!rushAngleOk) { // rush is not allowed, align to target first
                lateraler.update(0);
                stepper.update(0);
                turner.update(vecCap);
            } else { // rush toward the target while aligning toward it
                stepper.update(vecX);
                lateraler.update(0);
                turner.update(vecCap);
            }
        } else { // classic mode
            //logger.log("Direct mode is disabled");
            stepper.update(vecX);
            lateraler.update(vecY);
            turner.update(vecAzimuth);
        }
    }

    if (dontWalk) {
        walk->control(false);
    } else {
        // Using mm to control walk
        walk->control(true, 1000 * stepper.output, 1000 * lateraler.output, turner.output);
    }

    // Sharing the target and temporary target
    auto teamPlay = getServices()->teamPlay;
    auto &info = teamPlay->selfInfo();
    info.targetX = targetX;
    info.targetY = targetY;
    info.localTargetX = tmpX;
    info.localTargetY = tmpY;

    bind->push();
}
