#include "PlacementOptimizer.h"

void PlacementOptimizer::collectSolutions(
    std::map<int, int> robots,
    std::map<int, Target> targets,
    std::vector<PlacementOptimizer::Solution> &solutions,
    Solution partial
)
{
    for (auto &robot : robots) {
        for (auto &target : targets) {
            Solution solution = partial;
            solution.robotTarget[robot.second] = target.second;
            
            if (robots.size() == 1 || targets.size() == 1) {
                solutions.push_back(solution);
            } else {
                auto tmpRobots = robots;
                tmpRobots.erase(robot.first);
                auto tmpTargets = targets;
                tmpTargets.erase(target.first);
                
                collectSolutions(tmpRobots, tmpTargets, solutions, solution);
            }
        }    
    }
}

PlacementOptimizer::Solution PlacementOptimizer::optimize(
    std::vector<int> robots,
    std::vector<Target> targets,
    std::function<float (PlacementOptimizer::Solution)> scoreFunc)
{
    std::vector<Solution> solutions;
    
    Solution bestSolution;
    bool hasBest = false;
    float bestScore = 0;
    
    size_t k;
    
    std::map<int, int> robotsMap;
    k = 0;
    for (auto &robot : robots) {
        robotsMap[k++] = robot;    
    }
    
    std::map<int, Target> targetsMap;
    k = 0;
    for (auto &target : targets) {
        targetsMap[k++] = target;
    }
    collectSolutions(robotsMap, targetsMap, solutions);
    
    for (auto &solution : solutions) {
        float score = scoreFunc(solution);
        
        if (!hasBest || score < bestScore) {
            hasBest = true;
            bestScore = score;
            bestSolution = solution;    
        }
    }
    
    return bestSolution;
}