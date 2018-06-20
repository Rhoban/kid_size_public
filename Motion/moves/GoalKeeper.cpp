#include "GoalKeeper.h"
#include <math.h>


#include "Placer.h"
#include "Walk.h"
#include <services/LocalisationService.h>
#include <services/TeamPlayService.h>
#include <services/StrategyService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>
#include <services/TeamPlayService.h>
#include "Playing.h"
#include "Head.h"
#include "rhoban_utils/logging/logger.h"

static rhoban_utils::Logger logger("GoalKeeper");

static const bool DIVING=false;

#define STATE_STARTWAIT "start_wait"
#define STATE_GOHOME "gohome"   // go back to home position
#define STATE_STANDING "standing" // stop without block position when ball is far away
#define STATE_STOP "stop"      // stop and take the block position
#define STATE_ALIGNBALL "align" // align goalkeeper with ball in x position
#define STATE_ATTACK "attack"   // go to ball and shoot it out
#define STATE_DIVING_LEFT  "diving_left"
#define STATE_DIVING_RIGHT "diving_right"

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace rhoban_team_play;
using robocup_referee::Constants;
// xxx hysteresis sur align trop grande.
// Prévoir mouvement d'arrêt
// checker odométrie

/*
See GoalKeeper.svg for a schematic
 */

GoalKeeper::GoalKeeper(Walk *walk, Placer *placer)
  : walk(walk),  placer(placer) , neverWalked(true){
  initializeBinding();

  bind->bindNew("xAttack", xAttack, RhIO::Bind::PullOnly)
      ->comment("if ball in x margin regarding to goal area, attack [m]")
      ->defaultValue(0.8);
  bind->bindNew("yAttack", yAttack, RhIO::Bind::PullOnly)
      ->comment("if ball in y margin regarding to goal area, attack [m]")
      ->defaultValue(0.5);

  bind->bindNew("xAttackHys", xAttackHys, RhIO::Bind::PullOnly)
      ->comment("xAttack hysteresis (value added to xAttack) [m]")
      ->defaultValue(0.25);
  bind->bindNew("yAttackHys", yAttackHys, RhIO::Bind::PullOnly)
      ->comment("yAttack hysteresis (value added to yAttack) [m]")
      ->defaultValue(0.15);



  bind->bindNew("homeX", homeX, RhIO::Bind::PullOnly)
      ->comment("Distance to goal line in meters")
      ->defaultValue(0.5);

  bind->bindNew("maxHomeDistance", maxHomeDistance, RhIO::Bind::PullOnly)
      ->comment("Distance to home position acceptable to wait (in front of homeX line) [m]")
      ->defaultValue(0.3);

  bind->bindNew("maxHomeDistanceHys", maxHomeDistanceHys, RhIO::Bind::PullOnly)
      ->comment("Distance to home position acceptable to wait (in front of homeX line) hysteresis [m]")
      ->defaultValue(0.1);

  bind->bindNew("nextStateSize", nextStateSize, RhIO::Bind::PullOnly)
    ->comment("length of the buffer used to smooth next state change")
    ->defaultValue(3);

  //bind->bindNew("isPlacing", isPlacing, RhIO::Bind::PushOnly)
  //    ->comment("Is the robot performing an accurate placement")
  //    ->defaultValue(false)
  //    ->persisted(true);

  bind->bindNew("stopPosture", stopPosture, RhIO::Bind::PullOnly)
    ->comment("Is the robot adopt a special posture when stopped")
    ->defaultValue(true)
    ->persisted(true);
  
  bind->bindNew("stopMoveTime", stopMoveTime, RhIO::Bind::PullOnly)
      ->comment("Duration of the up/down move [sec]")
      ->defaultValue(1.5)
      ->persisted(true);
  
  bind->bindNew("xIgnoreBall", xIgnoreBall, RhIO::Bind::PullOnly)
      ->comment("Ignore ball if ball is out of xIgnoreBall position [m]")
      ->defaultValue(4)
      ->persisted(true);
  
  bind->bindNew("xIgnoreBallHys", xIgnoreBallHys, RhIO::Bind::PullOnly)
      ->comment("xIgnoreBallHys Hys position [m]")
      ->defaultValue(0.5)
      ->persisted(true);

  bind->bindNew("alignTolerance", alignTolerance, RhIO::Bind::PullOnly)
      ->comment("consider is align if distance with optimal point is below"
                " this value (is added to placer tolerance) [m]")
      ->defaultValue(0.4)
      ->persisted(true);

  bind->bindNew("maxShootDist", maxShootDist, RhIO::Bind::PullOnly)
      ->comment("adversary maximum shoot distance [m]")
      ->defaultValue(4)
      ->persisted(true);
  
  bind->bindNew("againstGrassRatio", againstGrassRatio, RhIO::Bind::PullOnly)
      ->comment("ratio to applied when opponent are against grass []")
      ->defaultValue(0.6)
      ->persisted(true);

  bind->bindNew("targetX", targetX, RhIO::Bind::PushOnly)->comment("Target X [m]");
  bind->bindNew("targetY", targetY, RhIO::Bind::PushOnly)->comment("Target Y [m]");


  bind->bindNew("t", t, RhIO::Bind::PushOnly)
    ->comment("Duration of the current state");

}

std::string GoalKeeper::getName() { return "goal_keeper"; }

void GoalKeeper::onStart() {
  //isPlacing = false;
  // Making the head move by default
  bind->pull();
  double g=RhIO::Root.getFloat("/strategy/grassOffset");
  if (g<1) opponentWithGrass=true;
  else opponentWithGrass=false;

  if (opponentWithGrass==false){
    grassRatio=againstGrassRatio;
  } else grassRatio=1.0;    
  
  initElbowOffsetValue=RhIO::Root.getFloat("/moves/walk/elbowOffset");
  initArmsRollValue=RhIO::Root.getFloat("/moves/walk/armsRoll");
  initTrunkZOffsetValue=RhIO::Root.getFloat("/moves/walk/trunkZOffset");
  logger.log("GK: initial values %f %f %f\n",initElbowOffsetValue,initArmsRollValue,initTrunkZOffsetValue);
  setTeamPlayState(Playing);
  Head *head = (Head *)getMoves()->getMove("head");
  head->setDisabled(false);
  setState(STATE_STARTWAIT);
  nextState.resize(nextStateSize);
  nextStateIndice=0;
  //RhIO::Root.setBool("/lowlevel/left_knee/torqueEnable", true);
  //RhIO::Root.setBool("/lowlevel/right_knee/torqueEnable", true);
  auto loc = getServices()->localisation;
  loc->isGoalKeeper(true);
  // set the margin to increase placement accuracy
  RhIO::Root.setFloat("/moves/placer/marginX",0.2);
  RhIO::Root.setFloat("/moves/placer/marginY",0.2);
  // increase head pan to 145 (135 by default)
  RhIO::Root.setFloat("/moves/head/maxPanTrack",145);
  RhIO::Root.setFloat("/moves/head/maxPan",145);
  // when ball is located, keep an eye on it
  RhIO::Root.setFloat("/moves/head/nearTrackDist",xIgnoreBall*grassRatio);
  RhIO::Root.setFloat("/moves/head/nearTrackingPeriod",45);  
}

/* avoid changing state too often...
*/
void GoalKeeper::bufferedSetState(const std::string &s){
  nextState[nextStateIndice]=s;
  nextStateIndice=(nextStateIndice+1)%nextState.size();
  bool ok=true;
  //logger.log("%d %d %s",nextStateIndice,nextState.size(),s.c_str());
  for(auto i=nextState.begin()+1;i!=nextState.end();++i){
    if (*i != *nextState.begin()){
      ok=false;
      return;
    }
  }
  if (ok){
    setState(s);
  }
}

void GoalKeeper::onStop() {
  // TODO stop the placer too
  setTeamPlayState(Inactive);
  auto &strategy = getServices()->strategy;
  stopMove("placer", 0.0);
  stopMove(strategy->getDefaultApproach(), 0.0);
  stopMove("clearing_kick_controler", 0.0);
  logger.log("GK: ***** ON STOP ***** ");
  //loc->enableFieldFilter(true);
  if (state==STATE_STOP)
    RhIO::Root.setBool("/moves/walk/gkMustRaise",true);     
  RhIO::Root.setBool("/moves/walk/gkMustBlock",false);
  setState(STATE_STANDING);
  /*
  float z=RhIO::Root.getFloat("/moves/walk/trunkZOffset");
  while(z>initTrunkZOffsetValue){
    RhIO::Root.setFloat("/moves/walk/trunkZOffset", z);
    z-=0.001;
    usleep(100);
  }
  RhIO::Root.setFloat("/moves/walk/armsRoll", initArmsRollValue); 
  RhIO::Root.setFloat("/moves/walk/elbowOffset", initElbowOffsetValue);
  */
}

TeamPlayState GoalKeeper::teamState() {
  return getServices()->teamPlay->selfInfo().state;
}

// check that the ball is in the goalAreaLength +xd +yd
bool GoalKeeper::ballInZone(float xd, float yd) {
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto decision = getServices()->decision;
  float lineX = -Constants::field.fieldLength / 2 + Constants::field.goalAreaLength + xd;
  float lineY = Constants::field.goalAreaWidth / 2 + yd;

  //logger.log("Current ball: %f %f", ball.x, ball.y);
  //logger.log("Limits: %f %f", lineX, lineY);

  return decision->isBallQualityGood && ball.x < lineX && fabs(ball.y) < lineY;
}

bool GoalKeeper::ballInAttackZone() {
  auto loc = getServices()->localisation;
  auto pos = loc->getFieldPos();
  auto ball = loc->getBallPosField();
  return ballInZone(xAttack, -yAttack) || (pos.getDist(ball)<0.5);
}

bool GoalKeeper::ballInAttackZoneHysteresis() {
  return ballInZone(xAttack+xAttackHys,-yAttack-yAttackHys);
}

bool GoalKeeper::ignoreBall() {
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto decision = getServices()->decision;

  return !(decision->isBallQualityGood && (ball.x < (-Constants::field.fieldLength/2 + xIgnoreBall*grassRatio)));
}

bool GoalKeeper::ignoreBallHys() {
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto decision = getServices()->decision;
  return !(decision->isBallQualityGood && (ball.x < (-Constants::field.fieldLength/2 + xIgnoreBall*grassRatio + xIgnoreBallHys)));
}

Point GoalKeeper::home(){
  return Point(-(Constants::field.fieldLength/2)+homeX,0);
}

// return a point on the backline that is suppose to be the shooting target of the opponent
Point GoalKeeper::shootLineCenter(){
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  float dx=ball.x - (-(Constants::field.fieldLength/2.0f)); // dx = distance to backline
  if ((dx-maxShootDist*grassRatio) > -0.05) {// ball is too far
    //logger.log("ball is too far %f %f %f",dx,dx-maxShootDist,ball.y);
    return Point(-Constants::field.fieldLength/2.0f,ball.y); // go in front
  }
  float dy=maxShootDist*grassRatio*maxShootDist*grassRatio - dx*dx;
  if (dy<0){
    //logger.log("error shootLineCenter: %f %f(%f) %f",maxShootDist,dx,ball.x,dy);
    // should not happen!
    return Point(-Constants::field.fieldLength/2.0f,ball.y); // go in front
  }
  dy=sqrtf(dy);
  float up=ball.y+dy;
  float down=ball.y-dy;
  up=std::min(up,(float)(Constants::field.goalWidth/2.0f));
  down=std::max(down,(float)(-Constants::field.goalWidth/2.0f));
  return Point(-Constants::field.fieldLength/2.0f,(up+down)/2.0f);
}


bool GoalKeeper::isNearHome(){
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();
    //logger.log("is Near Home ?  %f > %f && %f <= %f  >>> %d ",pos.x , (-(Constants::field.fieldLength/2)+homeX) , distToHome , maxHomeDistance,(((pos.x > (-(Constants::field.fieldLength/2)+homeX)) && (distToHome<=maxHomeDistance))));
    //logger.log("field orientation: %f",loc->getFieldOrientation());
    return ( (pos.x > (-(Constants::field.fieldLength/2)+homeX))
	     && (home().getDist(pos)<=maxHomeDistance)
	     && (fabs(loc->getFieldOrientation()*180.0/3.14)<20)
	     );
}

bool GoalKeeper::isNearHomeHys(){
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();


    return (
	    (pos.x > (-(Constants::field.fieldLength/2)+homeX))
	    && (home().getDist(pos)<=(maxHomeDistance+maxHomeDistanceHys))
	    && (fabs(loc->getFieldOrientation()*180.0/3.14)<25)
	    );
}

Point intersect(const Point  &a, const Point &b, float x){
  float slop=(b.y-a.y)/(b.x - a.x);
  float shift=a.y-slop*a.x;
  return Point(x,slop*x+shift);
}

Point securePoint(const Point &p){
  return Point(p.x,std::max(std::min(p.y,Constants::field.goalWidth/2.0f),-Constants::field.goalWidth/2.0f));
}

Point GoalKeeper::getAlignPoint(const Point &ref,float &theta){
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto pos = loc->getFieldPos();


  if ((ball.x) < -Constants::field.fieldLength) { // ball is in the goal ? go back to home
    theta=0;
    return securePoint(ref);
  }


  float a=-Constants::field.fieldLength/2+homeX;
  float b=-Constants::field.fieldLength/2+homeX+maxHomeDistance;
  float x=pos.x;
  if ((x<a) ||(x>b)) // correct x value
    x=(a+b)/2;

  //compute  y value on x coordinate using line: ref / ball
  Point d=intersect(ref,ball,x);
  float v=fabs(d.y)/(Constants::field.goalWidth/2.0);
  v=std::min(v,1.0f);
  float corFactor= (-(2*v-1)*(2*v-1)+1 ) * 0.15; // CHECK THIS
  //float tmp=d.y;
  if (d.y>0) d.y+=corFactor;
  else d.y-=corFactor;
  //logger.log("correction : %f %f %f  ==> %f",v,corFactor,tmp,d.y);
  theta = (ball-d).getTheta().getSignedValue();
  return securePoint(d);
}

bool GoalKeeper::isAligned(){
  auto loc = getServices()->localisation;
  auto pos = loc->getFieldPos();
  float t;

  auto ball = loc->getBallPosField();
  float dx=ball.x - (-(Constants::field.fieldLength/2.0f));
  //logger.log("isAligned: ball is at %f %f, dx is %f",ball.x,ball.y,dx);
  if (dx < 0) {    
    return true;
  }
  
  //logger.log("is aligned? %f < %f ",getAlignPoint(home(),t).getDist(pos),alignTolerance+placer->getMaxMarginXY());
  
  Point p = getAlignPoint(shootLineCenter(),t);
  double d = p.getDist(pos);

  if (d < (alignTolerance+placer->getMaxMarginXY())) {
    return true;
  }
  return false;
}

static float getLinear(float x,float x1, float y1,float x2, float y2){
  if (x1==x2){
    return y2;
  }
  if (y1==y2) return y1;
  float a=(y1-y2)/(x1-x2);
  return x*a+y2-x2*a;
}

void GoalKeeper::step(float elapsed) {
  bind->pull();
  t += elapsed;
  //logger.log("step0: %s t is %f (%f)",state,t,elapsed);
  if (state==STATE_STARTWAIT) {
    if (t<1.0) return;
    setState(STATE_GOHOME);
    return;
  }

  auto loc = getServices()->localisation;
  auto decision = getServices()->decision;  

  if (DIVING){  
    if (state==STATE_DIVING_RIGHT){
      RhIO::Root.setBool("/lowlevel/right_shoulder_roll/torqueEnable", true);
      RhIO::Root.setFloat("/lowlevel/right_shoulder_roll/torqueLimit", 1);
      setAngle("right_shoulder_roll",-160);
      if (t>0.5){
	setAngle("left_knee",80);
	setAngle("left_ankle_pitch",-45);
      }
      if (t>4){
	setState(STATE_GOHOME);
      }
      return;
    }

    if (state==STATE_DIVING_LEFT){    
      RhIO::Root.setBool("/lowlevel/left_shoulder_roll/torqueEnable", true);
      RhIO::Root.setFloat("/lowlevel/left_shoulder_roll/torqueLimit", 1);    
      setAngle("left_shoulder_roll",160);
      if (t>0.5){
	setAngle("right_knee",80);
	setAngle("right_ankle_pitch",-45);
      }
      if (t>4){
	setState(STATE_GOHOME);
      }
      return;
    }
  }
  
  
  if (state==STATE_STOP){
    
    if (stopPosture){
      if (t<stopMoveTime){
	float z=RhIO::Root.getFloat("/moves/walk/trunkZOffset");
	RhIO::Root.setFloat("/moves/walk/trunkZOffset", getLinear(t+elapsed,t,z,stopMoveTime,0.17));
      }
      else {
	// robot is stopped and down: try to anticipate a shoot for a possible jump
	Point future_ball_loc;
	for(int i=1;i<4;++i){
	  TimeStamp kick_time = TimeStamp::now()
	    + std::chrono::duration<int,std::milli>((int)(i * 1000));
	  future_ball_loc = loc->getPredictedBallSelf(kick_time);
	  logger.log("futur ball position at %d sec is %f %f ",i,future_ball_loc.x,future_ball_loc.y);
	}
	if (future_ball_loc.x<0) {
	  logger.log("ball is arriving!!! %f %f",future_ball_loc.x,future_ball_loc.y);
	  if (DIVING){	
	    if (future_ball_loc.y>0.15){
	      setState(STATE_DIVING_LEFT);
	      return;
	    }
	    if (future_ball_loc.y<-0.15){
	      setState(STATE_DIVING_RIGHT);
	      return;
	    }
	  }
	}
      }
      if ((t>0.8) && (t<stopMoveTime)){ // wait for 0.8s before opening arms
	RhIO::Root.setFloat("/moves/walk/elbowOffset", 0);
	RhIO::Root.setFloat("/moves/walk/armsRoll", 20);	
      }
      
    }
  } 

  //logger.log("step: %s t is %f (%f)",state,t,elapsed);
  
  // first, attack ball if necessary
  if (ballInAttackZone() && decision->isBallQualityGood){
    if (state!=STATE_ATTACK){
      logger.log("step: ball is in attack zone =>  attack!");
      bufferedSetState(STATE_ATTACK);
    }
    return;
  }

  if ((state==STATE_ATTACK) && (ballInAttackZoneHysteresis())){
    //logger.log("step: ball is in attack zone (hyst) =>  leave attack!");
    //setState(STATE_ATTACK); // stay in attack
    return;
  }

  if ((state==STATE_ATTACK) && !decision->isBallQualityGood){
    logger.log("step: ball Q is not good enough => leave attack!");
    bufferedSetState(STATE_STANDING);
    return;
  }

  // not in attack mode
  if (ignoreBall()){ // ball is not visible or out
    if (state==STATE_GOHOME){
      if (placer->arrived){
        logger.log("step: arrived to home : standing!");
        bufferedSetState(STATE_STANDING);
        return;
      }
    } else{
      if (state!=STATE_STANDING)
        bufferedSetState(STATE_GOHOME);
    }
  } else { // ball is visible but not close enought to attack => align wih it
    if (state==STATE_STOP){
      if (isAligned()==false)
        bufferedSetState(STATE_ALIGNBALL);
      // else stay stopped!
    } else {
      if (state!=STATE_ALIGNBALL){
        logger.log("step: ball is visible and we are not align: align with it!");
        bufferedSetState(STATE_ALIGNBALL);
      } else {
        if ((placer->arrived) && (state!=STATE_STOP))
          bufferedSetState(STATE_STOP);
      }
      float t;
      auto ball = loc->getBallPosField();
      float dx=ball.x - (-(Constants::field.fieldLength/2.0f));
      if (dx > 0) {
        auto p=getAlignPoint(shootLineCenter(),t);
	logger.log("placer => move to %f %f %f",p.x,p.y,t);
        placer->goTo(p.x,p.y,t);
      }

    }
  }

  bind->push();
}

void GoalKeeper::enterState(std::string state) {
  bind->pull();
  auto &strategy = getServices()->strategy;
  t = 0.0;
  // This seems to break the whole localisation system. Maybe since the
  // separation loc/vision? TODO: check this
  // auto loc = getServices()->localisation;
  // loc->enableFieldFilter(state != STATE_PLACE && placer->arrived);


  logger.log("ENTER STATE %s ",state.c_str());

  if (DIVING){
    if ((state==STATE_DIVING_RIGHT) || (state==STATE_DIVING_LEFT)){      
      RhIO::Root.setBool("/moves/walk/gkMustBlock",true);
    }
  }
  
  if (state==STATE_GOHOME){
    placer->goTo(home().x , home().y , 0);
    if ((neverWalked==true) && (placedByHand==false)){
      placer->setDirectMode(true);
    } else{
      placer->setDirectMode(false);
    }
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();

    if (neverWalked && home().getDist(pos)<0.1) {
      //setState(STATE_STOP);
      setState(STATE_STANDING);
    } else {
      startMove("placer", 0.0);
    }

  } else if (state==STATE_ATTACK){
    startMove(strategy->getDefaultApproach(), 0.0);
    startMove("clearing_kick_controler", 0.0);
  } else if (state==STATE_ALIGNBALL){
    //placer->setTemporaryMarginAzimuth(5);
    placer->setDirectMode(false);
    startMove("placer",0.0);    
  } else if (state==STATE_STANDING) {
    stopMove("placer",0.0);
  }else if (state==STATE_STOP){
    if (stopPosture){           
      //RhIO::Root.setFloat("/moves/walk/armsRoll", 20);
      //RhIO::Root.setFloat("/moves/walk/elbowOffset", 0);      
      RhIO::Root.setBool("/moves/walk/gkMustRaise",false);
    }
    neverWalked=false;
  }
}

void GoalKeeper::exitState(std::string state) {

  logger.log("LEAVE STATE %s after %f secs",state.c_str(), t);

  if (DIVING){
    if ((state==STATE_DIVING_RIGHT) || (state==STATE_DIVING_LEFT)){      
      RhIO::Root.setBool("/moves/walk/gkMustBlock",false);	
      RhIO::Root.setBool("/lowlevel/right_shoulder_roll/torqueEnable", false);
      RhIO::Root.setFloat("/lowlevel/right_shoulder_roll/torqueLimit", 0.2);
      RhIO::Root.setBool("/lowlevel/left_shoulder_roll/torqueEnable", false);
      RhIO::Root.setFloat("/lowlevel/left_shoulder_roll/torqueLimit", 0.2);
    }
  }
  
  if (state==STATE_STARTWAIT){
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();
    neverWalked=true;
    logger.log("home().x, home().y, pos.x, pos.y, home().getDist(pos)");
    logger.log("Start wait : %f %f %f %f %f", home().x, home().y, pos.x, pos.y, home().getDist(pos));
    if (home().getDist(pos)<0.1){
      placedByHand=true;
    } else {
      placedByHand=false;
    }
  } else if (state==STATE_GOHOME){
    stopMove("placer", 0.0);
    placer->setDirectMode(true);
  } else if (state==STATE_ALIGNBALL){
    stopMove("placer", 0.0);
    placer->setDirectMode(true);
    //placer->restoreMarginAzimuth();
  } else if (state==STATE_STOP){
    if (stopPosture){
      RhIO::Root.setBool("/moves/walk/gkMustRaise",true);
      //startMove("walk");
      //RhIO::Root.setBool("/lowlevel/left_knee/torqueEnable", true);  
      //RhIO::Root.setBool("/lowlevel/right_knee/torqueEnable", true);  
      //RhIO::Root.setFloat("/moves/walk/trunkZOffset", initTrunkZOffsetValue);   
      //RhIO::Root.setFloat("/moves/walk/elbowOffset", initElbowOffsetValue);
      //RhIO::Root.setFloat("/moves/walk/armsRoll", initArmsRollValue);
    }
  } else if (state==STATE_ATTACK){
    auto &strategy = getServices()->strategy;
    stopMove(strategy->getDefaultApproach(), 0.0);
    stopMove("clearing_kick_controler", 0.0);
  }
}

void GoalKeeper::setTeamPlayState(TeamPlayState state)
{
    getServices()->teamPlay->selfInfo().state = state;
}
