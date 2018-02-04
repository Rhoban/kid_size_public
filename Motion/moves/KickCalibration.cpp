
#include "KickCalibration.hpp"
#include <math.h>
#include <services/LocalisationService.h>
#include "Approach.h"
#include <scheduler/MoveScheduler.h>
#include <json/json.h>
#include <util.h>
#include <string>

#include "KickController.h"

KickCalibration::KickCalibration(Approach *approach)
  : approach(approach), kickController(NULL)
{
  initializeBinding();

  bind->bindNew("startX", startX, RhIO::Bind::PullOnly)
    ->comment("X Offset for the starting")->defaultValue(75)->persisted(true);
  
  bind->bindNew("started", started, RhIO::Bind::PullOnly)
    ->comment("Start approach")->defaultValue(false);
}

std::string KickCalibration::getName(){
  return "KickCalibration";
}

void KickCalibration::onStart(){

  kickController = dynamic_cast<KickController *>(getScheduler()->getMove("kick_controler"));
  
  startMove("Head2");
  startMove("kick_controler");
  startMove("walk");
  startMove("approach_potential");
  
  T = 0;

  nbShoots = 0;
  
  currentState = "place";
  lastState = "place";

  logging = false;
  tLogging = 0;

  smallKick = false;
  normalKick = false;
  lateralKick = false;

  kickCap = 0.;
}

void KickCalibration::onStop(){
  
  stopMove("Head2");
  stopMove("kick_controler");
  stopMove("walk");  
  stopMove("approach_potential");
  
}

void KickCalibration::step(float elapsed){
  
  bind->pull();
  
  T += elapsed; 

  if(!logging){
  
    currentState = getMoves()->getMove("approach_potential")->getStatus(); 

    if(currentState.compare("shoot") == 0 && lastState.compare("place") == 0){// Starting to shoot
      
      smallKick = kickController->isKickSmall(); 
      lateralKick = kickController->isKickLateral();
      
      if(lateralKick)
	   std::cout <<"LATERALKICK" << std::endl;
      
	 if(!smallKick && !lateralKick)
	   normalKick = true;
	 else
	   normalKick = false;
      
      auto loc = getServices()->localisation; 
      Angle kickDirField = kickController->getKickDir();
      Angle playerDir(rad2deg(loc->getFieldOrientation()));
      Angle kickCapAngle = kickDirField - playerDir;
      kickCap = kickCapAngle.getValue();
      
    }
    
    if(currentState.compare("place") == 0 && lastState.compare("shoot") == 0){// Just shot
      std::cout << "Just shot" << std::endl;
      
      // std::cout << "is kick small? : " << kickController->isKickSmall() << std::endl;
      
      stopMove("approach_potential");      

      tLogging = 0;
      logging = true;
      nbShoots++;
      
    }
    
    lastState = currentState;

    
  }
  else{//logging

    tLogging += elapsed;
    std::cout << "Logging ... " << std::endl;
    
    if(tLogging > 2.){
      auto loc = getServices()->localisation;      
      auto ballPos = loc->getBallPosSelf();
    
      ballDistance = ballPos.getLength()*100;
      ballX = ballPos.x*100;
      ballY = ballPos.y*100;

      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("ballDistance", ballDistance));
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("ballX", ballX));
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("ballY", ballY)); 
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("smallKick", smallKick)) ;
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("lateralKick", lateralKick)); 
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("normalKick", normalKick));
      ballMap[std::to_string(nbShoots)].insert(std::pair<std::string, float>("kickCap", kickCap));


      toFile(ballMap, "testBallMap.json");

      logging = false;
      startMove("approach_potential");      
    }
    
  }
  
  bind->push();
}



void KickCalibration::toFile(std::map<std::string, std::map<std::string,  float>> &map, std::string filename){

  
  Json::Value json(Json::objectValue);
  Json::StyledWriter writer; 
  
  for (auto &entry : map) {
    
    auto &key = entry.first;
    auto &map2 = entry.second;
    
    json[key] = Json::Value(Json::objectValue);
    
    for (auto &entry2 : map2) {
      
      auto &key2 = entry2.first;
      auto &val = entry2.second;
      
      json[key][key2] = val;
      
    } 
    
  }
  
  std::string data;
  
  data = writer.write(json);
  
  file_put_contents(filename, data);

}
