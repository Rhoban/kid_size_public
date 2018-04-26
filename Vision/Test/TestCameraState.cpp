#include "TestCameraState.hpp"

#include "CameraState/CameraState.hpp"

namespace Vision {
namespace Utils {

double TestCameraState::hRobot = 0.6;

void TestCameraState::_run() {
  testBasics();
  testRobot2Camera();
  testCamera2Img();
  testRobotPosFromImg();
  testImgFromPanTilt();
}

// Test based on vector values with zero angles
void TestCameraState::testBasics() {
  CameraState cs(hRobot, Angle(0), Angle(0), Angle(0));
  // getHeight = hRobot
  assertEqualsDelta(cs.getHeight(), hRobot, TestCase::deltaDouble);

  cv::Mat camDir, robotDir;
  // Test 1 : camX = - robotY
  {
    camDir = cv::Mat::zeros(4, 1, CV_32F);
    camDir.at<float>(0) = 1;
    robotDir = cs.robotDirFromCamDir(camDir);
    assertEqualsDelta((double)robotDir.at<float>(0), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(1), -1.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(2), 0.0,
                      TestCase::deltaDouble);
  }
  // Test 2 : camY = - robotZ
  {
    camDir = cv::Mat::zeros(4, 1, CV_32F);
    camDir.at<float>(1) = 1;
    robotDir = cs.robotDirFromCamDir(camDir);
    assertEqualsDelta((double)robotDir.at<float>(0), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(1), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(2), -1.0,
                      TestCase::deltaDouble);
  }
  // Test 3 : camZ = robotX
  {
    camDir = cv::Mat::zeros(4, 1, CV_32F);
    camDir.at<float>(2) = 1;
    robotDir = cs.robotDirFromCamDir(camDir);
    assertEqualsDelta((double)robotDir.at<float>(0), 1.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(1), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)robotDir.at<float>(2), 0.0,
                      TestCase::deltaDouble);
  }
}

void TestCameraState::testRobot2Camera() {
  // USING ONLY TILT
  // Test: PanTilt pos of an object at hRobot in front of the robot
  // Object should be directly in front of the camera
  {
    CameraState cs(hRobot, Angle(45), Angle(0), Angle(0));
    cv::Mat posRobot = cv::Mat::zeros(4, 1, CV_32F);
    posRobot.at<float>(0) = hRobot;
    posRobot.at<float>(3) = 1;
    cv::Mat posCamera = cs.camFromRobot(posRobot);
    double dist = sqrt(2) * hRobot;
    assertEqualsDelta((double)posCamera.at<float>(0), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)posCamera.at<float>(1), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)posCamera.at<float>(2), dist,
                      TestCase::deltaDouble);
  }
  // USING TILT + PAN
  // Test: Same as above, but Pan is not null
  {
    CameraState cs(hRobot, Angle(45), Angle(90), Angle(0));
    cv::Mat posRobot = cv::Mat::zeros(4, 1, CV_32F);
    posRobot.at<float>(1) = hRobot;
    posRobot.at<float>(3) = 1;
    cv::Mat posCamera = cs.camFromRobot(posRobot);
    double dist = sqrt(2) * hRobot;
    assertEqualsDelta((double)posCamera.at<float>(0), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)posCamera.at<float>(1), 0.0,
                      TestCase::deltaDouble);
    assertEqualsDelta((double)posCamera.at<float>(2), dist,
                      TestCase::deltaDouble);
  }
}

void TestCameraState::testCamera2Img() {
  CameraState cs(hRobot, Angle(0), Angle(0), Angle(0));
  cv::Mat camPos;
  cv::Point2i imgPos;
  int imgWidth(800), imgHeight(600);
  // if x = 0 and y = 0 then object should be near image center
  {
    camPos = cv::Mat::zeros(4, 1, CV_32F);
    camPos.at<float>(2) = 1;
    camPos.at<float>(3) = 1;
    imgPos = cs.posImgFromCamPos(camPos, imgWidth, imgHeight);
    assertTrue(abs(imgPos.x - imgWidth / 2) < 2);
    assertTrue(abs(imgPos.y - imgHeight / 2) < 2);
  }
  // if x > 0, then position x in image should be higher than imgWidth /2
  {
    camPos = cv::Mat::zeros(4, 1, CV_32F);
    camPos.at<float>(0) = 1;
    camPos.at<float>(2) = 1;
    camPos.at<float>(3) = 1;
    imgPos = cs.posImgFromCamPos(camPos, imgWidth, imgHeight);
    assertTrue(imgPos.x > imgWidth / 2);
    assertTrue(abs(imgPos.y - imgHeight / 2) < 2);
  }
  // if y > 0, then position y should be higher than imgHeight / 2
  {
    camPos = cv::Mat::zeros(4, 1, CV_32F);
    camPos.at<float>(1) = 1;
    camPos.at<float>(2) = 1;
    camPos.at<float>(3) = 1;
    imgPos = cs.posImgFromCamPos(camPos, imgWidth, imgHeight);
    assertTrue(abs(imgPos.x - imgWidth / 2) < 2);
    assertTrue(imgPos.y > imgHeight / 2);
  }
}

void TestCameraState::testRobotPosFromImg() {
  cv::Point2f robotPos;
  // Object is in center of the image with a tilt = 45°
  //-> object is  at [hRobot, 0]
  {
    CameraState cs(hRobot, Angle(45), Angle(0), Angle(0));
    robotPos = cs.robotPosFromImg(400, 200, 800, 400);
    assertEqualsDelta((double)robotPos.x, hRobot, TestCase::deltaDouble);
    assertEqualsDelta((double)robotPos.y, 0, TestCase::deltaDouble);
  }
  // Object is in center of the image with a tilt = 45° and pan = 90°
  // -> object is at [0, hRobot]
  {
    CameraState cs(hRobot, Angle(45), Angle(90), Angle(0));
    robotPos = cs.robotPosFromImg(400, 200, 800, 400);
    assertEqualsDelta((double)robotPos.y, hRobot, TestCase::deltaDouble);
    assertEqualsDelta((double)robotPos.x, 0, TestCase::deltaDouble);
  }
}

void TestCameraState::testImgFromPanTilt() {
  int imgWidth = 800;
  int imgHeight = 400;
  // When robot is looking forward
  {
    CameraState cs(hRobot, Angle(0), Angle(0));
    // 0 pan and 0 tilt must be at center
    Angle pan(0), tilt(0);
    cv::Point imgPos = cs.imgXYFromRobotPanTilt(pan, tilt, imgWidth, imgHeight);
    assertTrue(imgWidth / 2 == imgPos.x);
    assertTrue(imgHeight / 2 == imgPos.y);
    // x pan and 90 tilt must be at x center
    imgPos = cs.imgXYFromRobotPanTilt(pan, tilt, imgWidth, imgHeight);
    assertTrue(imgWidth / 2 == imgPos.x);
  }
  // When robot is looking down
  {
    CameraState cs(hRobot, Angle(90), Angle(0));
    // 0 pan and 90 tilt must be at center
    Angle pan(0), tilt(90);
    cv::Point imgPos = cs.imgXYFromRobotPanTilt(pan, tilt, imgWidth, imgHeight);
    assertTrue(imgWidth / 2 == imgPos.x);
    assertTrue(imgHeight / 2 == imgPos.y);
    // x pan and 90 tilt must be at center
    pan = Angle(12);
    imgPos = cs.imgXYFromRobotPanTilt(pan, tilt, imgWidth, imgHeight);
    assertTrue(imgWidth / 2 == imgPos.x);
    assertTrue(imgHeight / 2 == imgPos.y);
  }
}
}
}
