#include <math.h>
#include "WalkTest.h"
#include "services/ModelService.h"

WalkTest::WalkTest()
{  
  Move::initializeBinding();

  bind->bindNew("trunkZOffset", engine.trunkZOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkZOffset);
  bind->bindNew("frequency", engine.frequency, RhIO::Bind::PullOnly)->defaultValue(engine.frequency);
  bind->bindNew("footYOffset", engine.footYOffset, RhIO::Bind::PullOnly)->defaultValue(engine.footYOffset);
  bind->bindNew("riseGain", engine.riseGain, RhIO::Bind::PullOnly)->defaultValue(engine.riseGain);
  bind->bindNew("swingGain", engine.swingGain, RhIO::Bind::PullOnly)->defaultValue(engine.swingGain);
  
  bind->bindNew("xSpeed", engine.xSpeed, RhIO::Bind::PullOnly)->defaultValue(0.0);
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
  engine.xSpeed = 0;
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
    engine.newStep();
    engine.update(walkT);
  }
  
  // Assigning to robot
  engine.assignModel(getServices()->model->goalModel());

  bind->push();
}
