#include "StandUp.h"

#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/serialization/json_serializable.h>

using rhoban_utils::Function;


static rhoban_utils::Logger logger("StandUp");

StandUp::StandUp()
{
    Move::initializeBinding();
    time=0.0;
    bind->bindNew("speed", speed)
        ->defaultValue(1.0)->minimum(0.0)->maximum(20.0)
        ->comment("Speed factor for standing up")
        ->persisted(true);
    bind->bindNew("enable", enable)
        ->defaultValue(true)->persisted(true);
    bind->bindNew("over", over, RhIO::Bind::PushOnly)
        ->comment("Is the move over?")
        ->defaultValue(true);
    bind->bindNew("trying", trying, RhIO::Bind::PushOnly)
        ->comment("Number of try")
        ->defaultValue(0);

    bind->bindNew("delayBefore", delayBefore, RhIO::Bind::PullOnly)
        ->comment("Delay before standing up")
        ->defaultValue(2);

    bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)
        ->defaultValue(10)->persisted(true)->comment("Arms roll");

    bind->bindNew("useManualT", useManualT, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Use manual time");

    bind->bindNew("manualT", manualT, RhIO::Bind::PullOnly)
        ->defaultValue(0)->comment("Manual T");
    
    bind->bindNew("layDown", layDown, RhIO::Bind::PullOnly)
        ->defaultValue(false);
}

StandUp::~StandUp()
{
}

std::string StandUp::getName()
{
    return "standup";
}

void StandUp::onStart()
{
    time = 0.0;
    over = false;
    waiting = true;
    trying = 0;

    //get the arms back
    setTorqueLimit("left_shoulder_pitch", 1.0);
    setTorqueLimit("right_shoulder_pitch", 1.0);
    setTorqueLimit("left_shoulder_roll", 1.0);
    setTorqueLimit("right_shoulder_roll", 1.0);
}

void StandUp::onStop()
{
    //security for arms. TODO should be a init command.
    setTorqueLimit("left_shoulder_pitch", 0.2);
    setTorqueLimit("right_shoulder_pitch", 0.2);
    setTorqueLimit("left_shoulder_roll", 0.2);
    setTorqueLimit("right_shoulder_roll", 0.2);
}
    
void StandUp::setLayDown(bool value)
{
    bind->node().setBool("layDown", value);
}

void StandUp::step(float elapsed)
{
    bind->pull();

    if (waiting) {
        time += elapsed;
        if (useManualT || time > delayBefore) {
            try { 
                if (layDown) {
                    splines = Function::fromFile("lay_down.json");
                } else {
                    if (getPitch() < 0) {
                        splines = Function::fromFile("standup_back.json");
                    } else {
                        splines = Function::fromFile("standup_front.json");
                    }
                }
            } catch (const rhoban_utils::JsonParsingError & exc) {
                logger.error("%s", exc.what());
            }

            waiting = false;
            time = 0;
        }
    } else {
        if (enable) {
            float finalSpeed = speed*(1/(1+trying/2.0));

            if (finalSpeed < 0.5) finalSpeed = 0.5;
            time += elapsed*finalSpeed;
            if (useManualT) {
                time = manualT;
            }

            setAngle("left_shoulder_roll", armsRoll);
            setAngle("right_shoulder_roll", -armsRoll);
            setAngle("head_pitch", splines["head_pitch"].get(time));
            setAngle("left_shoulder_pitch", splines["shoulder_pitch"].get(time));
            setAngle("right_shoulder_pitch", splines["shoulder_pitch"].get(time));
            setAngle("left_elbow", splines["elbow"].get(time));
            setAngle("right_elbow", splines["elbow"].get(time));
            setAngle("left_hip_pitch", splines["hip_pitch"].get(time));
            setAngle("right_hip_pitch", splines["hip_pitch"].get(time));
            setAngle("left_knee", splines["knee"].get(time));
            setAngle("right_knee", splines["knee"].get(time));
            setAngle("left_ankle_pitch", splines["ankle_pitch"].get(time));
            setAngle("right_ankle_pitch", splines["ankle_pitch"].get(time));
            over = splines["over"].get(time) > 0.5;

            if (useManualT) {
                over = false;
            }
        } else {
            over = true;
        }
    }

    bind->push();
}
