#include <set>
#include <random>
#include <iostream>
#include <cmath>
#include "KickQLearning.hpp"
#include <robocup_referee/constants.h>
#include <rhoban_geometry/segment.h>
#include <rhoban_geometry/circle.h>

using namespace robocup_referee;
using namespace rhoban_geometry;

KickQLearning::KickQLearning(
        std::string kickFiles,
        double accuracy,
        double angleAccuracy,
        double goalieWidth,
        bool enableExcentric,
        bool dump,
        double tolerance,
        double grassOffset,
        double penaltyMultiplier,
        std::string corridorProfilePath
        )
    :
        accuracy(accuracy),
        angleAccuracy(angleAccuracy),
        goalieWidth(goalieWidth),
        enableExcentric(enableExcentric),
        dump(dump),
        tolerance(tolerance),
        penaltyMultiplier(penaltyMultiplier)
{
    kicks.loadFile(kickFiles);
    kicks.setGrassConeOffset(grassOffset);
    if (corridorProfilePath != "") {
      corridorProfile.loadFile(corridorProfilePath);
    }
}

double KickQLearning::rewardFor(State *from, State *state)
{
    if (state == &successState) {
        return 0;
    }

    if (state == &failState) {
        return -100;
    }

    //double fX = accuracy*from->x;
    //double fY = accuracy*from->y;
    double X = accuracy*state->x;
    double Y = accuracy*state->y;
    double multiplier = corridorProfile.getWeight(X,Y);

    if (X > (Constants::field.fieldLength-Constants::field.goalAreaLength) &&
            fabs(Y-Constants::field.fieldWidth/2) < Constants::field.goalAreaWidth) {
        multiplier = penaltyMultiplier;
    }

    if (enableExcentric) {
        bool ok = false;
        double limitA = Constants::field.fieldLength*0.5;
        double yOff = fabs(Y-Constants::field.fieldWidth/2);
        if (X < limitA) {
            ok = (yOff < Constants::field.goalWidth/2);
        } else {
            /*
            double dX = X - limitA;
            ok = (yOff < Constants::field.goalWidth/2 - dX*1.5);
            */
        }
        /*
        if (yOff > Constants::field.fieldWidth/2 - 0.25) {
            ok = true;
        }
        */
        if (ok) {
            multiplier = 3;
        }
    }

    double dist = sqrt(
            pow(from->x*accuracy - state->x*accuracy, 2) +
            pow(from->y*accuracy - state->y*accuracy, 2)
            );

    return -multiplier*(15 + dist/0.15);
}

KickStrategy KickQLearning::generate()
{
    KickStrategy strategy(accuracy);

    // Generate the states
    generateStates();

    // Genrate kick templates
    generateTemplate();

    // Generate the model of actions for each state
    generateModels();

    // Iterate over all the possibilities
    while(iterate());

    for (int y=0; y<ySteps; y++) {
        for (int x=0; x<xSteps; x++) {
            double X = accuracy*x;
            double Y = accuracy*y;

            Action action;
            double bestScore = -1000;
            double bestLength = -1;
            std::set<int> possibleOrientations;
            std::set<std::string> possibleKicks;
            std::map<std::string, double> kickLength;
            for (auto &kickName : kicks.getKickNames()) {
//                if (kickName == "small") continue;
                auto &kickModel = kicks.getKickModel(kickName);
                auto tmp = kickModel.applyKick(Eigen::Vector2d(0, 0), 0);
                kickLength[kickName] = tmp[0];
            }

            for (auto &entry : states[x][y].models) {
                double actionScore = 0;
                auto tmpAction = entry.first;

                for (auto &possibility : entry.second) {
                    actionScore +=
                        possibility.first*(
                                rewardFor(&states[x][y], possibility.second) +
                                possibility.second->score
                                )
                        ;
                }

                double tmpTol = tolerance;
                if (X < Constants::field.fieldLength/2) tmpTol *= 2;
                if (fabs(actionScore - states[x][y].score) < tmpTol) {
                    possibleKicks.insert(tmpAction.kick);
                    possibleOrientations.insert(round(tmpAction.orientation*1000));
                    auto length = kickLength[tmpAction.kick];

                    if (bestScore < actionScore || bestLength < 0 || bestLength < length) {
                        bestLength = length;
                        action = tmpAction;
                        bestScore = actionScore;
                    }

                    if (dump) {
                        // Debug draw of the kick arrows
                        double R = 0.1;
                        double kickX = X + cos(tmpAction.orientation)*R;
                        double kickY = Y + sin(tmpAction.orientation)*R;

                        std::cout << X << " " << Y << " " << actionScore << std::endl;
                        std::cout << kickX << " " << kickY << " " << actionScore  << std::endl;
                        std::cout << std::endl << std::endl;
                    }
                }

            }

            KickStrategy::Action resultAction;
            // XXX: We suppose that the possible orientations are dispatched evenly
            resultAction.tolerance = std::max<double>(0, (possibleOrientations.size()-1)*angleAccuracy*M_PI/180.0);
            resultAction.orientation = action.orientation;
            resultAction.score = bestScore;

            if (possibleKicks.count("classic") && possibleKicks.count("lateral")) {
                resultAction.kick = "opportunist";
            } else {
                resultAction.kick = action.kick;
            }

            strategy.setAction(X, Y, resultAction);
        }
    }

    return strategy;
}

void KickQLearning::generateStates()
{
    xSteps = 1+Constants::field.fieldLength/(accuracy);
    ySteps = 1+Constants::field.fieldWidth/(accuracy);
    aSteps = 360/angleAccuracy;

    successState.score = 0;
    failState.score = 0;

    for (int x=0; x<xSteps; x++) {
        for (int y=0; y<ySteps; y++) {
            State state;
            state.x = x;
            state.y = y;
            state.score = -1000;
            states[x][y] = state;
        }
    }
}

void KickQLearning::generateTemplate()
{
    std::default_random_engine generator;
    std::normal_distribution<double> posNoise(0, 0.3);

#define SAMPLES 10000

    for (int a=0; a<aSteps; a++) {
        for (auto &kickName : kicks.getKickNames()) {
//            if (kickName == "small") continue;
            auto &kickModel = kicks.getKickModel(kickName);
            Action action;
            action.kick = kickName;
            action.orientation = (a*2*M_PI)/aSteps;

            std::map<std::pair<int, int>, int> count;

            for (size_t k=0; k<SAMPLES; k++) {
                auto result = kickModel.applyKick(Eigen::Vector2d(0, 0), action.orientation, &generator);
                auto sample = Point(result[0], result[1]);

                int dX = round(sample.x/accuracy) + posNoise(generator);
                int dY = round(sample.y/accuracy) + posNoise(generator);

                count[std::pair<int,int>(dX, dY)] += 1;
            }

            for (auto &entry : count) {
                kickTemplate[action].push_back(PossibilityTemplate(entry.second/(double)SAMPLES, entry.first));
            }
        }
    }
}

void KickQLearning::generateModels()
{
    std::default_random_engine generator;
    std::normal_distribution<double> rnd(0, 1);

    for (int x=0; x<xSteps; x++) {
        for (int y=0; y<ySteps; y++) {
            for (auto &entry : kickTemplate) {
                auto action = entry.first;
                Model model;
                std::map<State*, double> count;
                double X = accuracy*x;
                double Y = accuracy*y;

                for (auto &possibilityTpl : entry.second) {
                    double p = possibilityTpl.first;
                    auto pos = possibilityTpl.second;
                    double kickX = X + accuracy*(pos.first);
                    double kickY = Y + accuracy*(pos.second);
                    auto state = stateFor(kickX, kickY);

                    if (state != NULL) {
                        count[state] += p;
                    } else {
                        if (kickX > Constants::field.fieldLength) {
                            double dY = (kickY-Y)/(kickX-X);
                            double yIntersect = Y + (Constants::field.fieldLength-X)*dY;
                            yIntersect -= Constants::field.fieldWidth/2;

                            // Goal
                            if (fabs(yIntersect) < goalieWidth/2.0) {
                                double goalProbability = 0.25;

                                auto goalFeet = stateFor(Constants::field.fieldLength - 0.1,
                                                         Constants::field.fieldWidth/2 + yIntersect);

                                count[goalFeet]
                                    += p*goalProbability;

                                count[&successState] += p*(1-goalProbability);

                            } else if (fabs(yIntersect) < (Constants::field.goalWidth*0.95)/2) {
                                count[&successState] += p;
                            } else {
                                count[&failState] += p;
                            }
                        } else {
                            count[&failState] += p;
                        }
                    }
                }

                for (auto &entry : count) {
                    model.push_back(Possibility(entry.second, entry.first));
                }

                states[x][y].models[action] = model;
            }
        }
    }
}

bool KickQLearning::iterate()
{
    bool changed = false;

    for (int x=0; x<xSteps; x++) {
        for (int y=0; y<ySteps; y++) {
            double score = -5000;
            for (auto &entry : states[x][y].models) {
                double actionScore = 0;
                for (auto &possibility : entry.second) {
                    actionScore +=
                        possibility.first*(
                                rewardFor(&states[x][y], possibility.second) +
                                possibility.second->score
                                );
                }

                if (actionScore > score) {
                    score = actionScore;
                }
            }

            if (score > states[x][y].score) {
                changed = true;
                states[x][y].score = score;
            }
        }
    }

    return changed;
}

KickQLearning::State *KickQLearning::stateFor(double x, double y)
{
    int X = round(x/accuracy);
    int Y = round(y/accuracy);

    if (states.count(X)) {
        if (states[X].count(Y)) {
            return &states[X][Y];
        }
    }

    return NULL;
}
