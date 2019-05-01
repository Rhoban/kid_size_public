#include <math.h>
#include "WalkTest.h"
#include "services/ModelService.h"

WalkTest::WalkTest()
{  
  Move::initializeBinding();

  bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("trunkPitch", trunkPitch, RhIO::Bind::PullOnly)->defaultValue(0);

  bind->bindNew("trunkXOffset", engine.trunkXOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkXOffset);
  bind->bindNew("trunkZOffset", engine.trunkZOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkZOffset);
  bind->bindNew("frequency", engine.frequency, RhIO::Bind::PullOnly)->defaultValue(engine.frequency);
  bind->bindNew("footYOffset", engine.footYOffset, RhIO::Bind::PullOnly)->defaultValue(engine.footYOffset);
  bind->bindNew("riseGain", engine.riseGain, RhIO::Bind::PullOnly)->defaultValue(engine.riseGain);
  bind->bindNew("riseDuration", engine.riseDuration, RhIO::Bind::PullOnly)->defaultValue(engine.riseDuration);
  bind->bindNew("swingGain", engine.swingGain, RhIO::Bind::PullOnly)->defaultValue(engine.swingGain);
  
  bind->bindNew("xSpeed", engine.xSpeed, RhIO::Bind::PullOnly)->defaultValue(0.0);
  bind->bindNew("ySpeed", engine.ySpeed, RhIO::Bind::PullOnly)->defaultValue(0.0);
  bind->bindNew("yawSpeed", engine.yawSpeed, RhIO::Bind::PullOnly)->defaultValue(0.0);

  walkT = 0;
}

std::string WalkTest::getName()
{
  return "walk_test";
}

void WalkTest::onStart()
{
  Leph::HumanoidFixedModel& model = getServices()->model->goalModel();
  engine.initByModel(model);

  bind->node().setBool("walkEnable", false);

  bind->node().setFloat("xSpeed", 0.0);
  bind->node().setFloat("ySpeed", 0.0);
  bind->node().setFloat("yawSpeed", 0.0);

  engine.swingGain = 0;
  engine.riseGain = 0;
  engine.xSpeed = 0;
  engine.ySpeed = 0;
  engine.yawSpeed = 0;
  engine.reset();
}

void WalkTest::step(float elapsed)
{
  bind->pull();

  // Ticking
  walkT += elapsed;
  double over = engine.update(walkT);
  if (over > 0) {
    walkT = over;
    std::cout << "New step!" << std::endl;

    if (!walkEnable) {
      engine.swingGain = 0;
      engine.riseGain = 0;
      engine.xSpeed = 0;
      engine.ySpeed = 0;
      engine.yawSpeed = 0;
    }

    engine.newStep();
    engine.update(walkT);
  }

  // Assigning to robot
  engine.assignModel(getServices()->model->goalModel());

  ModelService* model = getServices()->model;
  model->flushLegs(_smoothing);

  // Applying extra trunk pitch
  setAngle("left_hip_pitch", trunkPitch);
  setAngle("right_hip_pitch", trunkPitch);

  bind->push();
}
