#include <math.h>
#include <services/DecisionService.h>
#include <services/LocalisationService.h>
#include <services/RefereeService.h>
#include <services/TeamPlayService.h>
#include <services/StrategyService.h>
#include <services/CaptainService.h>
#include <rhoban_utils/control/control.h>
#include <robocup_referee/constants.h>
#include "Head.h"
#include "Placer.h"
#include "Playing.h"
#include "Walk.h"
#include "Approach.h"
#include "rhoban_utils/logging/logger.h"

#define STATE_APPROACH  "approach"
#define STATE_WALKBALL  "walkBall"
#define STATE_SEARCH    "search"
#define STATE_LET_PLAY  "letPlay"
#define STATE_BACKWARD  "backward"
#define STATE_STOPPING  "stopping"
#define STATE_LOCALIZE  "localize"

static rhoban_utils::Logger logger("PlayingSTM");

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace rhoban_team_play;
using namespace robocup_referee;

PlayingMove::PlayingMove(Walk *walk)
    : walk(walk)
{
    initializeBinding();

    bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
        ->comment("State of the Playing STM");

    bind->bindNew("localizeWalkDuration", localizeWalkDuration, RhIO::Bind::PullOnly)
        ->defaultValue(10.0)->comment("Walk forward during this duration");

    bind->bindNew("useKickController", useKickController, RhIO::Bind::PullOnly)
        ->defaultValue(true)->comment("Use the kick controller");

    bind->bindNew("stopOnHandle", stopOnHandle, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Stop playing when handled (debug handling)");

    bind->bindNew("teamConfidence", teamConfidence, RhIO::Bind::PullOnly)
        ->defaultValue(true)->comment("Am I confident in my team?");

    bind->bindNew("walkBallDistance", walkBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(1.2)->comment("Distance to run the approach to the ball [m]");
        
    bind->bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)->comment("Radius [m] to avoid colliding the ball while placing");
}

std::string PlayingMove::getName()
{
    return "playing";
}

void PlayingMove::onStart()
{
    approach = (Approach*)getMoves()->getMove("approach");
    head = (Head*)getMoves()->getMove("head");
    placer = (Placer*)getMoves()->getMove("placer");
    head->setDisabled(false);
    setTeamPlayState(Playing);
    setState(STATE_SEARCH);

    // Ensuring fieldQThreshold is reasonable
    if (RhIO::Root.getValueFloat("decision/fieldQThreshold").value > 0.9) {
        logger.warning("High value for decision/fieldQThreshold");
    }
    
    backwardT = 10;
}

void PlayingMove::onStop()
{
    setState(STATE_STOPPING);
    setTeamPlayState(Inactive);
}

void PlayingMove::step(float elapsed)
{
    bind->pull();

    auto loc = getServices()->localisation;
    auto decision = getServices()->decision;
    auto teamPlay = getServices()->teamPlay;
    auto captain = getServices()->captain;
    auto instruction = captain->getInstruction();
    t += elapsed;

    if (decision->handled && stopOnHandle) {
        walk->control(false);
        stop();
    }

    if (state == STATE_LOCALIZE) {
        auto referee = getServices()->referee;

        // Walk forward when the robot enter the field
        if (referee->getTimeSincePlaying() < localizeWalkDuration) {
            walk->control(true, walk->maxStep);
        } else {
            // Behavior depend on the time spent in the state
            // Robot repeat a simple loop:
            // - wait (Robot is scanning)
            // - rotate
            // - wait
            // - walk forward
            
            // All of this could be parameters
            double locWait = 4;//[s]
            double locRotTime = 6;//[s]
            double locWalkTime = 10;//[s]
            double localizePeriod = 2 * locWait + locRotTime + locWalkTime;
            
            // Avoid moving at full speed there (less risky)
            double walkSpeed = 0.7 * walk->maxStep;
            double turnSpeed = 0.7 * walk->maxRotation;
            double affix = fmod(t, localizePeriod);
            
            if (affix < locWait) {
                walk->control(false);
            } else if (affix < locWait + locRotTime) {
                walk->control(true, 0, 0, turnSpeed);
            } else if (affix < 2 * locWait + locRotTime) {
                walk->control(false);
            } else{
                walk->control(true, walkSpeed);
            }
        }

        // Once quality is good enough, go to search state
        if (decision->isFieldQualityGood && !loc->getVisualCompassStatus()) {
            setState(STATE_SEARCH);
        }
    } else {
        // We are approaching the ball
        if (state == STATE_APPROACH || state == STATE_WALKBALL) {
            auto ball = loc->getBallPosSelf();
            auto dist = ball.getLength();

            if (state == STATE_APPROACH) {
                if (dist > walkBallDistance*1.1) {
                    setState(STATE_WALKBALL);
                }
            }

            if (state == STATE_WALKBALL) {
                if (dist < walkBallDistance && decision->isBallQualityGood) {
                    setState(STATE_APPROACH);
                }
                
                // Defend inflection
                Point ballField;
                if (decision->isBallQualityGood) {
                    ballField = loc->getBallPosField();
                } else {
                    ballField = Point(instruction.ball.x, instruction.ball.y);
                }
                auto goalField = loc->getOurGoalPosField();
                double c = -ballField.x*2/Constants::field.fieldLength;
                if (c < 0) {
                    c = 0;
                }
                c *= dist;
                if (c > walkBallDistance*0.85) {
                    c = walkBallDistance*0.85;
                }
                auto ballGoalField = (goalField-ballField).normalize(c);
                ballGoalField += ballField;
                
                placer->goTo(ballGoalField.x, ballGoalField.y, 0);
            }

            if (!decision->isBallQualityGood && state == STATE_APPROACH) {
                setState(STATE_BACKWARD);
            }
        } else {
            if (!decision->shouldLetPlay && instruction.order == CaptainOrder::HandleBall) {
                setState(STATE_WALKBALL);
            }
        }

        // We are letting the other play
        if (state == STATE_LET_PLAY) {
            if (!teamConfidence || instruction.order != CaptainOrder::Place) {
                // Getting ball distance
                float letPlayRadius = decision->letPlayRadius;
                auto ball = loc->getBallPosField();
                auto ballPos = loc->getBallPosSelf();
                float ballDistance = ballPos.getLength();
                float ballAzimuth = ballPos.getTheta().getSignedValue();

                Point goalCenter(-Constants::field.fieldLength/2, 0);
                double fieldOrientation = rad2deg(loc->getFieldOrientation());
                float defendAzimuth = (Angle(fieldOrientation) - (ball-goalCenter).getTheta()).getSignedValue();

                // Walking to the let play radius
                Control stepper;
                stepper.min = -walk->maxStepBackward;
                stepper.max = walk->maxStep;
                stepper.k_p = 100;
                stepper.update(ballDistance-letPlayRadius);

                // Aligning with the ball
                Control aligner;
                aligner.min = -walk->maxRotation;
                aligner.max = walk->maxRotation;
                aligner.k_p = 0.2;
                aligner.update(ballAzimuth);

                // Move to block the goals
                Control lateraler;
                lateraler.min = -walk->maxLateral;
                lateraler.max = walk->maxLateral;
                lateraler.k_p = 10;
                if (fabs(ballAzimuth) > 15 || fabs(ballDistance-letPlayRadius) > 0.35) {
                    defendAzimuth = 0;
                }
                lateraler.update(defendAzimuth);

                walk->control(true, stepper.output, lateraler.output, aligner.output);
                
                if (placer->isRunning()) {
                    placer->stop();
                }
            } else {
                // XXX: Some values should be Rhiozed here
                Point ball, ballTarget;
                ball = Point(decision->shareX, decision->shareY);
                ballTarget = Point(decision->ballTargetX, decision->ballTargetY);

                // Ball handler
                // XXX: The captain ball handler could be used instead
                bool handlerFound = false;
                TeamPlayInfo handler;
                handler.fieldX = 0;
                handler.fieldY = 0;
                for (auto &entry : teamPlay->allInfo()) {
                    if (entry.second.state == BallHandling) {
                        handlerFound = true;
                        handler = entry.second;
                    }
                }

                auto target = instruction.targetPosition;
                auto orientation = instruction.targetOrientation;
                if (!placer->isRunning()) {
                    placer->start();
                }

                // Avoiding the ball
                std::vector<Circle> obstacles;
                obstacles.push_back(Circle(ball.x, ball.y, avoidRadius));
                
                // Avoiding the ball trajectory
                // Point tmp = ball;
                // Point kickVect = (ballTarget - ball);
                // double norm = kickVect.getLength();
                // int parts = norm/0.3;
                // if (parts > 5) parts = 5;
                // for (int k=1; k<=parts; k++) {
                //     Point pos = ball + kickVect.normalize(norm*k/(float)parts);
                //     obstacles.push_back(Circle(pos.x, pos.y, 0.5));
                // }
                
                if (handlerFound) {
                    obstacles.push_back(Circle(handler.fieldX, handler.fieldY, avoidRadius));
                }
                placer->goTo(target.x, target.y, orientation,
                        obstacles);
            }

            if (!teamConfidence) {
                if (!decision->isBallQualityGood) {
                    logger.log("I don't see the ball, going to search");
                    setState(STATE_SEARCH);
                }
            }
        } else {
            if (decision->shouldLetPlay || instruction.order == CaptainOrder::Place) {
                // No confidence in the team, we should see the ball to enter let play,
                // else we are in search state
                if (decision->isBallQualityGood) {
                    logger.log("Letting play");
                    setState(STATE_LET_PLAY);
                } else if (teamConfidence && decision->ballIsShared) {
                    // We have confidence in the team, even if the ball is shared we
                    // still enter this state and avoid searching for it
                    logger.log("Letting play (confident)");
                    setState(STATE_LET_PLAY);
                }
            }
        }
        
        if (decision->isBallQualityGood) {
            backwardT = 0;
        } else {
            backwardT += elapsed;
        }

        if (state == STATE_SEARCH || state == STATE_BACKWARD) {
            logger.log("Let's approach the ball, I see it");
            /*
            if (decision->isBallQualityGood) {
                logger.log("Let's approach the ball, I see it");
                setState(STATE_APPROACH);
            }
            */
        } else {
            if (instruction.order == CaptainOrder::SearchBall && !decision->isBallQualityGood) {
                if (backwardT < 3) {
                    setState(STATE_BACKWARD);
                } else {
                    setState(STATE_SEARCH);
                }
            }
        }

        // We are walking backward
        if (state == STATE_BACKWARD) {
            walk->control(true, -walk->maxStepBackward, 0, 0);

            if (t > 5.0) {
                setState(STATE_SEARCH);
            }
        }

        // When field quality is low or when the robot is running visual
        // compass: go to state localize
        if (!decision->isFieldQualityGood || loc->getVisualCompassStatus()) {
            setState(STATE_LOCALIZE);
        }
    }

    bind->push();
}

void PlayingMove::enterState(std::string state)
{
    logger.log("Entering state %s", state.c_str());
    auto &strategy = getServices()->strategy;

    if (state == STATE_LOCALIZE) {
        head->setForceLocalize(true);
    } else {
        head->setForceLocalize(false);
    }

    if (state == STATE_APPROACH) {
        startMove(strategy->getDefaultApproach(), 0.0);
    }

    if (state == STATE_APPROACH || state == STATE_WALKBALL || state == STATE_LET_PLAY) {
        if (useKickController) {
            startMove(strategy->getDefaultKickController(), 0.0);
        }
    }

    if (state == STATE_APPROACH || state == STATE_WALKBALL) {
        setTeamPlayState(BallHandling);
    }

    if (state == STATE_SEARCH) {
        startMove("search", 0.0);
    }

    if (state == STATE_WALKBALL) {
        startMove("placer", 0.0);
    }

    t = 0;
}

void PlayingMove::exitState(std::string state)
{
    logger.log("Exiting state %s", state.c_str());
    auto &strategy = getServices()->strategy;

    if (state == STATE_APPROACH) {
        stopMove(strategy->getDefaultApproach(), 0.0);
    }
    
    if (state == STATE_APPROACH || state == STATE_WALKBALL || state == STATE_LET_PLAY) {
        stopMove(strategy->getDefaultKickController(), 0.0);
    }

    if (state == STATE_BACKWARD || state == STATE_LOCALIZE || state == STATE_LET_PLAY) {
        walk->control(false);
    }
    
    if (state == STATE_APPROACH || state == STATE_WALKBALL || state == STATE_LET_PLAY) {
        setTeamPlayState(Playing);
    }

    if (state == STATE_LET_PLAY || state == STATE_WALKBALL) {
        stopMove("placer", 0.0);
    }

    if (state == STATE_SEARCH) {
        stopMove("search", 0.0);
    }
    
    // Avoid glitching off the walk during transition between placing and approach
    if ((state == STATE_WALKBALL && nextState == STATE_APPROACH) || 
        (state == STATE_APPROACH && nextState == STATE_WALKBALL)) {
        walk->control(true);
    }
}

void PlayingMove::setTeamPlayState(TeamPlayState state)
{
    getServices()->teamPlay->selfInfo().state = state;
}
