#include "PushRecovery3.h"

#include <scheduler/MoveScheduler.h>
#include "Walk.h"
#include <RhAL.hpp>

#include <math.h>
#include <RhIO.hpp>
#include <rhoban_utils/angle.h>
#include <Model/HumanoidFloatingModel.hpp>
#include <rhoban_utils/control/variation_bound.h>
#include "services/ModelService.h"
#include <cstring>

#include <string>

#include <vector>
#include "util.h"

#include <nominal/Nominal.h>


#include <limits>

static Nominal nominal(4, 30);  //TODO Essayer avec moins de bins, logger pendant plus longtemps

PushRecovery3::PushRecovery3(Walk *walk)
  :walk(walk)
{

  initializeBinding();
  
  bind->bindNew("pushWalkEnable", pushWalkEnable)
    ->comment("pushWalkEnable")->defaultValue(false)->persisted(true);

  bind->bindNew("enableReaction", enableReaction)
    ->comment("enableReaction")->defaultValue(false)->persisted(true);

  bind->bindNew("logTime", logTime, RhIO::Bind::PullOnly)
    ->comment("logTime")->defaultValue(30)->persisted(true);
  
  bind->bindNew("offsetTrunkPitch", offsetTrunkPitch, RhIO::Bind::PullOnly)
    ->comment("offsetTrunkPitch")->defaultValue(0)->persisted(true);

  
  bind->bindNew("offsetTrunkZ", offsetTrunkZ, RhIO::Bind::PullOnly)
    ->comment("offsetTrunkZ")->defaultValue(0.02)->persisted(true);
  
  bind->bindNew("thresholdNominalScore", thresholdNominalScore, RhIO::Bind::PullOnly)
    ->comment("thresholdNominalScore")->defaultValue(10)->persisted(true);
  
  // bind->bindNew("thresholdMeanAccX", thresholdMeanAccX, RhIO::Bind::PullOnly)
  //   ->comment("thresholdMeanAccX")->defaultValue(1)->persisted(true);
  
  // bind->bindNew("coeffStepSize", coeffStepSize, RhIO::Bind::PullOnly)
  //   ->comment("coeffStepSize")->defaultValue(1)->persisted(true);
  
  bind->bindNew("offsetHip", offsetHip, RhIO::Bind::PullOnly)
    ->comment("offsetHip")->defaultValue(25)->persisted(true);
  
  // bind->bindNew("offsetAnkle", offsetAnkle, RhIO::Bind::PullOnly)
  //   ->comment("offsetAnkle")->defaultValue(0)->persisted(true);
  
  bind->bindNew("enableLog", enableLog, RhIO::Bind::PullOnly)
    ->comment("enableLog")->defaultValue(false)->persisted(true);

}

std::string PushRecovery3::getName(){
  return "pushrecovery3";
}


void PushRecovery3::onStart(){
  bind->pull();

  startMove("walk");
  startMove("head");
  
  walk->control(pushWalkEnable);
  
  saved = false;
  loaded = false;
  t = 0;
  waiting = true;
  cooldown = false;
  cooldownTime = 0.;

  ready = true;
  recoverFall = false;

  leftTimeSinceTouchedGround = 0.;
  rightTimeSinceTouchedGround = 0.;

  tMove = 0; 
  perturbationLastIteration = false;

  normalTrunkPitchBackwards = walk->getTrunkPitchBackward();
  
  bind->push();
}


void PushRecovery3::onStop(){
  
  stopMove("walk");
  stopMove("head");
  
}

double getPeakQueue(std::deque<double> queue){

  double mean = 0;
  for(int i = 0 ; i < queue.size() ; i++){
    mean += queue[i];
  }

  mean /= queue.size();

  double peakMin = std::numeric_limits<double>::max();
  double peakMax = std::numeric_limits<double>::lowest();
  
  for(int i = 0 ; i < queue.size() ; i++){
    if(queue[i] > peakMax)
      peakMax = queue[i];
    if(queue[i] < peakMin)
      peakMin = queue[i];
  }
  
  return abs(peakMax) > abs(peakMin) ? peakMax : peakMin;
  
}


void PushRecovery3::step(float elapsed){
  bind->pull();
  
  walk->control(pushWalkEnable, 0, 0, 0);

  walk->setTrunkZOffset(offsetTrunkZ);
  // walk->setTrunkPitchBackward(normalTrunkPitchBackwards+offsetTrunkPitch);

  // std::cout << "getTrunkPitchBackward : " << walk->getTrunkPitchBackward() << std::endl;
  
  if(pushWalkEnable){
    
    if(enableLog){
      if(t < logTime+5){//logging
	t += elapsed;
	
	if(t>5)
	  waiting = false;
	
	if(!waiting){
	  
	  double accX = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccX();
	  double accY = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccY();
	  double pitch = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getPitch();
	
	  double pressureLeft = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getWeight();
	  double pressureRight = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getWeight();

	  double leftX = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getX();
	  double leftY = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getY();
	  double rightX = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getX();
	  double rightY = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getY();

	  Eigen::VectorXd v(4);
  	  
	  // v <<
	  //   pitch,
	  //   accX,
	  //   leftX,
	  //   rightX
	  //   ;

	  v <<
	    pitch,
	    accX,
	    pressureLeft,
	    pressureRight
	    ;
	  
	  nominal.add(walk->getPhase(), v);
	  std::cout << "Logging ..." << std::endl;
	}
	else
	  std::cout << "waiting ..." << std::endl;
      }
      else{//logging over 
	
	if(!saved){
	  std::cout << "not saved yet" << std::endl;
	  
	  nominal.compile();
	  std::cout << "compiled ok " << std::endl;
	  
	  nominal.saveToJson("LogsPushRecovery/testNominal.json");
	  
	  std::cout << "Saved to JSON " << std::endl;
	  saved = true;
	  std::cout << "DONE" << std::endl;
	  
	}
	  
	}	
      }
      else{//Not logging

	if(!loaded){
	
	  nominal.loadJson("LogsPushRecovery/testNominal.json");
	  loaded = true;
	}
	else{//Json Loaded
	
	  double accX = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccX();
	  double accY = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccY();
	  double pitch = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getPitch();
	
	  double pressureLeft = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getWeight();
	  double pressureRight = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getWeight();

	  double leftX = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getX();
	  double leftY = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure").getY();
	  double rightX = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getX();
	  double rightY = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure").getY();

	  meanAccXQueue.push_back(accX);

	  if(meanAccXQueue.size() > 500)
	    meanAccXQueue.pop_front();

	  double meanAccX = 0;
	  for(int i = 0 ; i < meanAccXQueue.size() ; i++)
	    meanAccX += meanAccXQueue[i];

	  meanAccX /= meanAccXQueue.size();
	
	  Eigen::VectorXd v(4);
	
	  // v <<
	  //   pitch,
	  //   accX,
	  //   leftX,
	  //   rightX
	  //   ;

	  v <<
	    pitch,
	    accX,
	    pressureLeft,
	    pressureRight
	    ;
	
	  double nominalScore = nominal.getBin(walk->getPhase()).score(v);
	
	  double nominalAccX = nominal.getBin(walk->getPhase()).mean[0];
	
	  // std::cout << std::endl << "nominal accX : " << nominalAccX << std::endl;
	  // std::cout << "current accX : " << accX << std::endl << std::endl;
	
	  if(cooldown){
	    cooldownTime += elapsed;
	    // if(cooldownTime<1 && enableReaction)
	    //   walk->control(pushWalkEnable, 25, 0, 0); 
	    std::cout << "coolown ..." << std::endl;
	  }
	  if(cooldownTime > 2.5){
	    if(!ready)
	      std::cout << "COOLDOWN DONE." << std::endl;
	    cooldownTime = 0.;
	    cooldown = false;
	    ready = true;
	  } 
	
	  // std::cout << "nominalScore : " << nominalScore << std::endl;
	  if(pressureLeft >= 30000){//Todo à tuner
	    leftTimeSinceTouchedGround = 0;
	    // std::cout << "left foot touched the ground" << std::endl;
	  }
	  else
	    leftTimeSinceTouchedGround+=elapsed;
	
	  if(pressureRight >= 10000){//Todo à tuner
	    rightTimeSinceTouchedGround = 0; 
	    // std::cout << "right foot touched the ground" << std::endl;
	  }
	  else
	    rightTimeSinceTouchedGround+=elapsed;

	  if(nominalScore > thresholdNominalScore && ready && !perturbationLastIteration)
	    perturbationLastIteration = true;
	
	  if(nominalScore > thresholdNominalScore && ready && perturbationLastIteration){
	    perturbationLastIteration = false;
	    std::cout << "PERTURBATION : " << nominalScore << std::endl;
	  
	    ready = false;

	    recoverFall = true;
	  
	  }
	  else if(!(nominalScore > thresholdNominalScore) && perturbationLastIteration)
	    perturbationLastIteration = false;
	
	  if(recoverFall){

	    std::cout << "Phase : " << walk->getPhase() << std::endl;

	    if(walk->getPhase() > 0.15 && walk->getPhase() < 0.35){// TODO Tuner seuils
	      std::cout << "right up" << std::endl;
	      triggerLeftFoot = false;
	      triggerRightFoot = true;
	      recoverFall = false;
	      angleHip = getAngle("left_hip_pitch");
	      angleAnkle = getAngle("left_ankle_pitch");
	    
	    
	    
	    }
	    else if(walk->getPhase() > 0.65 && walk->getPhase() <0.85){// TODO Tuner seuils
	      std::cout << "left up" << std::endl;
	      triggerLeftFoot = true;
	      triggerRightFoot = false;
	      recoverFall = false;
	      angleHip = getAngle("right_hip_pitch");	    
	      angleAnkle = getAngle("left_ankle_pitch");

	      
	    }
	  
	    // if(leftTimeSinceTouchedGround >= 0.7 && leftTimeSinceTouchedGround <= 0.21){
	    
	    //   triggerLeftFoot = true;
	    //   triggerRightFoot = false;
	    //   recoverFall = false;
	    //   angleHip = getAngle("left_hip_pitch");
	    
	    // }
	    // else if(rightTimeSinceTouchedGround >= 0.7 && rightTimeSinceTouchedGround <= 0.21){
	    //   triggerLeftFoot = false;
	    //   triggerRightFoot = true;
	    //   recoverFall = false;
	    //   angleHip = getAngle("right_hip_pitch");
	    // }

	  }

	  if(triggerLeftFoot){
	  
	    if(enableReaction){
	      setAngle("left_hip_pitch", angleHip+offsetHip);
	      std::cout << "normalTrunkPitchBackwards+offsetTrunkPitch :" << normalTrunkPitchBackwards+offsetTrunkPitch << std::endl; 
	      walk->setTrunkPitchBackward(normalTrunkPitchBackwards+offsetTrunkPitch);
	    }
	  

	  
	    tMove+=elapsed;
	    
	  }
	  else if(triggerRightFoot){
	    
	    if(enableReaction){
	      setAngle("right_hip_pitch", angleHip+offsetHip);
	      std::cout << "normalTrunkPitchBackwards+offsetTrunkPitch :" << normalTrunkPitchBackwards+offsetTrunkPitch << std::endl; 
	      walk->setTrunkPitchBackward(normalTrunkPitchBackwards+offsetTrunkPitch);

	    }
	    
	    tMove+=elapsed;
	    
	  }

	  if(tMove > 0.3){
	    
	    triggerLeftFoot = false;
	    triggerRightFoot = false;
	    tMove = 0;
	    cooldown = true;

	    walk->setTrunkPitchBackward(normalTrunkPitchBackwards);
	  }
	
	}
      
      }
    
    }
  
  bind->push();
}

Eigen::Vector3d PushRecovery3::nextStepPosition(){
  Eigen::Vector3d pos ;

  double accX = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccX();
  double accY = getScheduler()->getManager()->dev<RhAL::GY85>("imu").getAccY();

  Eigen::Vector2d xy ;
  xy <<
    accX,
    accY;

  

  
  return pos;
}


