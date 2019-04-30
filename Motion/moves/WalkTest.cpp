#include <math.h>
#include "WalkTest.h"
#include "services/ModelService.h"

WalkTest::WalkTest()
{  
  Move::initializeBinding();
  bind->bindNew("xSpeed", engine.xSpeed, RhIO::Bind::PullOnly)->defaultValue(0.0);
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
  double over = engine.update(elapsed);
  if (over > 0) {
    walkT = over;
    engine.newStep();
    engine.update(walkT);
  }
  
  // Assigning to robot
  engine.assignModel(getServices()->model->goalModel());

  bind->push();
}
