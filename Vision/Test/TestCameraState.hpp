#ifndef VISION_UTILS_TESTCAMERASTATE_HPP
#define VISION_UTILS_TESTCAMERASTATE_HPP

#include "tests/TestCase.h"

namespace Vision {
namespace Utils {

class TestCameraState : public TestCase {

public:
  static double hRobot;

protected:
  virtual void _run();

  void testBasics();
  void testRobot2Camera();
  void testCamera2Img();
  void testRobotPosFromImg();
  void testImgFromPanTilt();
};
}
}

#endif // VISION_UTILS_TESTCAMERASTATE_HPP
