#include <functional>
#include <algorithm>
#include <rhoban_geometry/segment.h>
#include <rhoban_geometry/circle.h>
#include <services/LocalisationService.h>
#include <services/ModelService.h>
#include <robocup_referee/constants.h>
#include "rhoban_utils/logging/logger.h"
#include "QKickController.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("QKickController");

QKickController::QKickController()
{
    Move::initializeBinding();

    bind->bindNew("strategyFile", strategyFile)
        ->defaultValue("kickStrategy_with_grass.json")
        ->comment("Strategy file")
        ->persisted(true);

    bind->bindNew("avoidOpponents", avoidOpponents, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Avoid the opponents?");

    bind->bindNew("farUpdateDist", farUpdateDist, RhIO::Bind::PullOnly)
        ->defaultValue(0.35)->persisted(true);

    bind->bindNew("moveUpdateDist", moveUpdateDist, RhIO::Bind::PullOnly)
        ->defaultValue(0.5)->persisted(true);

    bind->bindFunc("reloadStrategy",
        "Reload the strategy file",
        &QKickController::cmdReloadStrategy, *this);

    bind->pull();

    // Loading strategy
    if (!strategy.fromJson(strategyFile)) {
        logger.error("Can't load kick strategy file %s", strategyFile.c_str());
    }

    // Load available kicks
    kmc.loadFile();
}

std::string QKickController::cmdReloadStrategy()
{
    bind->pull();
    std::stringstream ss;

    if (!strategy.fromJson(strategyFile)) {
        ss << "Can't load strategy file " << strategyFile;
    } else {
        ss << "Strategy " << strategyFile << " loaded.";
    }

    return ss.str();
}

std::string QKickController::getName()
{
    return "q_kick_controler";
}

void QKickController::onStart()
{
    bind->pull();
    forceUpdate = true;
}

void QKickController::onStop()
{
}

void QKickController::updateAction()
{
    auto loc = getServices()->localisation;
    Point ball = loc->getBallPosField();
    action = strategy.actionFor(ball.x, ball.y);
    tolerance = rad2deg(action.tolerance)/2;

    if (action.kick == "") {
        return;
    }

    // XXX Captain: this should be removed once captain is implemented
    if (avoidOpponents) {
        auto opponentsPos = loc->getOpponentsField();

        if (opponentsPos.size()) {
            double opponentsRadius = loc->opponentsRadius;

            auto kick = action.kick;
            if (action.kick == "opportunist") {
                kick = "classic";
            }

            // Shoot segment and opponent
            std::vector<Circle> opponents;
            for (auto &opponentPos : opponentsPos) {
                Circle opponent(Point(opponentPos.x, opponentPos.y), opponentsRadius);
                opponents.push_back(opponent);
            }

            // Does the shoot intersects an opponent ?
            auto intersects = [&opponents, &ball](Point &point) -> bool {
                Segment shoot(ball, point);

                for (auto &opponent : opponents) {
                    if (shoot.intersects(opponent)) {
                        return true;
                    }
                }

                return false;
            };

            // Forcing the opponent to be at least at 110% its radius
            for (auto &opponent : opponents) {
                double limit = opponentsRadius * 1.1;
                auto tmp = opponent.getCenter()-ball;
                if (tmp.getLength() < limit) {
                    opponent.setCenter(opponent.getCenter() + tmp.normalize(limit));
                }
            }

            // Predicting the current shoot
            auto tmp = kmc.getKickModel(kick).applyKick(
                    Eigen::Vector2d(ball.x, ball.y), action.orientation
                    );
            Point predict(tmp[0], tmp[1]);

            // Checking if current shoot intersects the opponet
            if (intersects(predict)) {
                std::vector<KickStrategy::Action> candidates;
                std::vector<Point> tangents;

                // Tangents mode
                /*
                for (auto &opponent : opponents) {
                    auto tmp = opponent;
                    tmp.setRadius(tmp.getRadius()*1.1);
                     for (auto &tangent : tmp.tangents(ball)) {
                        tangents.push_back(tangent);
                    }
                }
                */

                // Brute force mode
                for (double alpha=-90; alpha<90; alpha+=5) {
                    double x = cos(deg2rad(alpha));
                    double y = sin(deg2rad(alpha));
                    tangents.push_back(ball + Point(x, y));
                }

                for (auto &tangent : tangents) {
                    auto vect = tangent-ball;
                    auto orientation = atan2(vect.y, vect.x);

                    bool classicOk = false;
                    bool lateralOk = false;
                    int classicId = 0, lateralId = 0;

                    for (auto &kickName : kmc.getKickNames()) {
                        if (kickName == "small") continue;
                        auto targetRes = kmc.getKickModel(kickName).applyKick(
                                Eigen::Vector2d(ball.x, ball.y), orientation
                                );
                        Point target(targetRes[0], targetRes[1]);

                        bool ok = true;
                        bool goal = false;

                        if (intersects(target)) {
                            // This new kick also intersects an opponent
                            ok = false;
                        } else if (target.x < ball.x) {
                            // The ball goes backward on the field
                            ok = false;
                        } else if (fabs(target.x) > Constants::field.fieldLength/2 ||
                                fabs(target.y) > Constants::field.fieldWidth/2) {
                            // Ball goes out of the field
                            ok = false;
                            if (target.x > Constants::field.fieldLength/2 &&
                                    ball.x <= Constants::field.fieldLength/2) {
                                double dY = (target.y-ball.y)/(target.x-ball.x);
                                double yIntersect = ball.y + (Constants::field.fieldLength/2-ball.x)*dY;

                                if (fabs(yIntersect) < Constants::field.goalWidth*0.8/2) {
                                    // But a goal is scored
                                    ok = true;
                                    goal = true;
                                }
                            }
                        }

                        if (ok) {
                            // Adding the action
                            KickStrategy::Action action;
                            action.kick = kickName;
                            action.orientation = orientation;
                            if (goal) {
                                action.score = target.x;
                            } else {
                                action.score = strategy.scoreFor(target.x, target.y);
                            }
                            if (action.kick == "classic") {
                                classicId = candidates.size();
                                classicOk = true;
                            }
                            if (action.kick == "lateral") {
                                lateralId = candidates.size();
                                lateralOk = true;
                            }
                            candidates.push_back(action);
                        }
                    }

                    if (classicOk && lateralOk) {
                        candidates[lateralId].kick = "opportunist";
                        candidates.erase(candidates.begin()+classicId);
                    }
                }

                if (candidates.size()) {
                    std::sort(candidates.begin(), candidates.end());
                    action = candidates[candidates.size()-1];
                    tolerance = 25;//Previously 10, increased to have faster
                                   //kick when there is an obstacle
                }
            }
        }
    }

    allowed_kicks.clear();
    if (action.kick == "opportunist") {
        allowed_kicks.push_back("classic");
        allowed_kicks.push_back("lateral");
    } else {
        allowed_kicks.push_back(action.kick);
    }

    kick_dir = rad2deg(action.orientation);
}

bool QKickController::shouldUpdate()
{
    auto loc = getServices()->localisation;
    Point ball = loc->getBallPosField();
    Point pos = loc->getFieldPos();
    double dist = (ball-pos).getLength();

    if (forceUpdate || dist > farUpdateDist || (ball-lastUpdateBall).getLength() > moveUpdateDist ) {
        lastUpdateBall = ball;
        forceUpdate = false;
        return true;
    }

    return false;
}

void QKickController::step(float elapsed)
{
    bind->pull();

    if (shouldUpdate()) {
        updateAction();
    }
}
