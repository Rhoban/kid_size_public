
#include "GoalKick.hpp"

#include <math.h>
#include <services/LocalisationService.h>
#include <scheduler/MoveScheduler.h>
#include <json/json.h>
#include <rhoban_utils/util.h>
#include <string>

using ::rhoban_utils::TimeStamp;
using namespace rhoban_geometry;

GoalKick::GoalKick()
{
  initializeBinding();

  // bind->bindNew("startX", startX, RhIO::Bind::PullOnly)
  //   ->comment("X Offset for the starting")->defaultValue(75)->persisted(true);
  
}

std::string GoalKick::getName(){
  return "GoalKick";
}

void GoalKick::onStart(){

  // kickController = dynamic_cast<KickController *>(getScheduler()->getMove("kick_controler"));
  
  startMove("Head2");
  // startMove("kick_controler");
  startMove("walk");
  
  currentBallDistance = 0.;
  previousBallDistance = 0.;

  previousBallPosition = Point(0, 0);
  currentBallPosition = Point(0, 0);
}

void GoalKick::onStop(){
  
  stopMove("Head2");
  stopMove("kick_controler");
  stopMove("walk");  
  
}

void GoalKick::step(float elapsed){
  
  bind->pull();
  
  auto loc = getServices()->localisation;

  auto now = TimeStamp::now();

  double doubleTS = now.getTimeMS();
  doubleTS+=1000;//TODO tune in function of the delay of the kick
  auto nowPlus1s = TimeStamp::fromMS(doubleTS);
  
  Point predictedBallSelf = loc->getPredictedBallSelf(nowPlus1s);

  (void) predictedBallSelf;
  




  
  // auto ballPos = loc->getBallPosSelf();
    
  // auto ballDistance = ballPos.getLength()*100;
  
  // auto ballX = ballPos.x*100;
  // auto ballY = ballPos.y*100;

  // currentBallPosition = Point(ballX, ballY);
  // currentBallDistance = ballDistance;

  // float ballSpeed = (previousBallDistance - currentBallDistance)/elapsed; 
  

  
  // std::cout << "Ball Speed : " << ballSpeed << std::endl;

  // std::cout << getBallDir(previousBallPosition, currentBallPosition) << std::endl;

  // Eigen::Vector2d vecBallDir = getBallDir(previousBallPosition, currentBallPosition);  
  
  // Eigen::Vector2d vecBallPos;
  
  // vecBallPos <<
  //   ballX,
  //   ballY
  //   ;

  // Eigen::Vector2d vecRobotPos;
  
  // vecRobotPos <<
  //   0.,
  //   0.
  //   ;

  // Eigen::Vector2d vecToPoint = vecBallPos - vecRobotPos;
  
  // // std::cout << vecToPoint.dot(vecBallDir) << std::endl; 
  // if(currentBallDistance != previousBallDistance){
  //   ballGettingCloser = vecToPoint.dot(vecBallDir) > 0 ? 1 : 0;
  //   std::cout << ballGettingCloser << std::endl;
  // }
  
  // previousBallPosition = currentBallPosition;
  // previousBallDistance = currentBallDistance;
  
  bind->push();
}

Eigen::Vector2d GoalKick::getBallDir(Point previousBallPos, Point currentBallPos){
  
  Eigen::Vector2d dir;
  
  dir <<
    previousBallPos.getX() - currentBallPos.getX(),
    previousBallPos.getY() - currentBallPos.getY()
    ;

  // std::cout << dir << std::endl;
  
  return dir;
}

// void GoalKick::toFile(std::map<std::string, std::map<std::string,  float>> &map, std::string filename){

  
//   Json::Value json(Json::objectValue);
//   Json::StyledWriter writer; 
  
//   for (auto &entry : map) {
    
//     auto &key = entry.first;
//     auto &map2 = entry.second;
    
//     json[key] = Json::Value(Json::objectValue);
    
//     for (auto &entry2 : map2) {
      
//       auto &key2 = entry2.first;
//       auto &val = entry2.second;
      
//       json[key][key2] = val;
      
//     } 
    
//   }
  
//   std::string data;
  
//   data = writer.write(json);
  
//   file_put_contents(filename, data);

// }
