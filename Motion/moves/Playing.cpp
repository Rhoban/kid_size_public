#include <math.h>
#include <services/DecisionService.h>
#include <services/LocalisationService.h>
#include <services/RefereeService.h>
#include <services/TeamPlayService.h>
#include <services/StrategyService.h>
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
    
    bind->bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)->comment("Radius [m] to avoid colliding the ball while placing");
    
    bind->bindNew("placingBallDistance", placingBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(2.0)->comment("Distance [m] to the ball for team play placement");
    
    bind->bindNew("perpendicularBallDistance", perpendicularBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(0.8)->comment("Distance [m] on the perpendicular to attack placement");

    bind->bindNew("walkBallDistance", walkBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(1.2)->comment("Distance to run the approach to the ball [m]");

    bind->bindNew("passPlacingRatio", passPlacingRatio, RhIO::Bind::PullOnly)
        ->defaultValue(0.85)->comment("Ratio to the kick vector to place");
    
    bind->bindNew("passPlacingOffset", passPlacingOffset, RhIO::Bind::PullOnly)
        ->defaultValue(0.35)->comment("Offset to kick vector to place [m]")->persisted(true);
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
}

void PlayingMove::onStop()
{
    setState(STATE_STOPPING);
    setTeamPlayState(Inactive);
}

static void boundPosition(Point &point)
{
    double xMax = Constants::field.fieldLength/2 - Constants::field.goalAreaLength - 0.5;
    double yMax = Constants::field.goalAreaWidth/2;

    if (point.x > xMax) {
        point.x = xMax;
        if (point.y > yMax) point.y = yMax;
        if (point.y < -yMax) point.y = -yMax;
    }
    if (point.x < -xMax) {
        point.x = -xMax;
        point.x = -xMax;
        if (point.y > yMax) point.y = yMax;
        if (point.y < -yMax) point.y = -yMax;
    }
}

double PlayingMove::score(Place place, Point pos)
{
    if (place.ok) {
        return (pos-place.position).getLength()/walk->maxStep;
    } else {
        return -1;
    }
}

PlayingMove::Place PlayingMove::findPlacingTarget(Point pos, Point ball, Point ballTarget,
        TeamPlayState intention)
{
    Place place;
    Point corner;
    Point target;
    Point vect = (ballTarget-ball)*passPlacingRatio 
        - (ballTarget-ball).normalize()*passPlacingOffset;
    Point nVect = vect.perpendicular().normalize(perpendicularBallDistance);
    bool backward = ballTarget.x-0.1 <= ball.x;

    if (intention == PlacingA) {
        corner = Point(Constants::field.fieldLength/2, Constants::field.goalWidth/2 + 0.25);

        if (backward) {
            target = ball+(ballTarget-ball)*1.15;
        } else {
            target = ball+vect+nVect;
        }
    } else if (intention == PlacingB) {
        corner = Point(Constants::field.fieldLength/2, -Constants::field.goalWidth/2 - 0.25);

        if (backward) {
            auto tmp1 = ball+vect+nVect;
            auto tmp2 = ball+vect+nVect;
            if (tmp1.x < tmp2.x) target = tmp1;
            else target = tmp2;
        } else {
            target = ball+vect-nVect;
        }
    } else if (intention == PlacingC) {
        corner = Point(-Constants::field.fieldLength/2, Constants::field.goalWidth/2 + 0.25);
        target = ball+(corner-ball).normalize(placingBallDistance);
    } else if (intention == PlacingD) {
        corner = Point(-Constants::field.fieldLength/2, -Constants::field.goalWidth/2 - 0.25);
        target = ball+(corner-ball).normalize(placingBallDistance);
    }

    auto oppositeCorner = corner;
    oppositeCorner.y *= -1;

    // Avoiding penalty areas
    boundPosition(target);

    // Avoiding being too close to the ball
    if ((target-ball).getLength() < avoidRadius) {
        place.ok = false;
    } else {
        place.ok = true;
    }

    // Target orientation
    Angle orientation;
    if (intention == PlacingA || intention == PlacingB) {
        orientation = (ballTarget-target).getTheta();
    } else {
        orientation = Angle::weightedAverage((ball-target).getTheta(), 0.5, 
            (oppositeCorner-target).getTheta(), 0.5);
    }

    // Building the place
    place.position = target;
    place.orientation = orientation;

    return place;
}

void PlayingMove::step(float elapsed)
{
    bind->pull();

    auto loc = getServices()->localisation;
    auto decision = getServices()->decision;
    auto teamPlay = getServices()->teamPlay;
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
                if (dist < walkBallDistance) {
                    setState(STATE_APPROACH);
                }
                auto ballField = loc->getBallPosField();
                placer->goTo(ballField.x, ballField.y, 0);
            }

            if (!decision->isBallQualityGood) {
                setState(STATE_BACKWARD);
            }
        } else {
            if (decision->isBallQualityGood && !decision->shouldLetPlay) {
                if (state == STATE_LET_PLAY) {
                    if (t > 1.5) { 
                        setState(STATE_WALKBALL);
                    }
                } else {
                    setState(STATE_WALKBALL);
                }
            }
        }

        // We are letting the other play
        if (state == STATE_LET_PLAY) {
            if (!teamConfidence || decision->iAmTheNearest) {
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
                stepper.k_p = 1;
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
                lateraler.k_p = 1;
                if (fabs(ballAzimuth) > 15 || fabs(ballDistance-letPlayRadius) > 0.35) {
                    defendAzimuth = 0;
                }
                lateraler.update(defendAzimuth);

                walk->control(true, stepper.output/100, lateraler.output/100, aligner.output);
                
                if (placer->isRunning()) {
                    placer->stop();
                }
            } else {
                // XXX: Some values should be Rhiozed here
                Point ball, ballTarget;
                ball = Point(decision->shareX, decision->shareY);
                ballTarget = Point(decision->ballTargetX, decision->ballTargetY);

                // My position
                auto pos = loc->getFieldPos();
                
                // My info
                auto &info = teamPlay->selfInfo();

                // Ball handler
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

                // Attacking target
                std::map<TeamPlayState, Place> places;
                for (auto &intention : {PlacingA, PlacingB, PlacingC, PlacingD}) {
                    places[intention] = findPlacingTarget(pos, ball, ballTarget, intention);
                }
                info.scoreA = score(places[PlacingA], pos);
                info.scoreB = score(places[PlacingB], pos);
                info.scoreC = score(places[PlacingC], pos);
                info.scoreD = score(places[PlacingD], pos);

                info.state = teamPlay->myRole();
                auto target = places[info.state];

                if (!placer->isRunning()) {
                    placer->start();
                }

                std::vector<Circle> obstacles;
                obstacles.push_back(Circle(ball.x, ball.y, avoidRadius));
                if (handlerFound) {
                    obstacles.push_back(Circle(handler.fieldX, handler.fieldY, avoidRadius));
                }
                placer->goTo(target.position.x, target.position.y, target.orientation.getSignedValue(),
                        obstacles);
            }

            if (teamConfidence) {
                if (!decision->isBallQualityGood && !decision->ballIsShared) {
                    logger.log("I don't see the ball, neither my team, going to search");
                    setState(STATE_SEARCH);
                }
            } else {
                if (!decision->isBallQualityGood) {
                    logger.log("I don't see the ball, going to search");
                    setState(STATE_SEARCH);
                }
            }
        } else {
            if (decision->shouldLetPlay) {
                // No confidence in the team, we should see the ball to enter let play,
                // else we are in search state
                if (decision->isBallQualityGood) {
                    logger.log("Respecting the radius and letting play");
                    setState(STATE_LET_PLAY);
                } else if (teamConfidence && decision->ballIsShared) {
                    // We have confidence in the team, even if the ball is shared we
                    // still enter this state and avoid searching for it
                    logger.log("Respecting the radius and letting play (confident)");
                    setState(STATE_LET_PLAY);
                }
            }
        }

        if (state == STATE_SEARCH || state == STATE_BACKWARD) {
            if (decision->isBallQualityGood) {
                logger.log("Let's approach the ball, I see it");
                setState(STATE_APPROACH);
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
}

void PlayingMove::setTeamPlayState(TeamPlayState state)
{
    getServices()->teamPlay->selfInfo().state = state;
}
