#include "PlacementOptimizer.h"

PlacementOptimizer::Target::Target()
: mandatory(false)
{    
}

void PlacementOptimizer::collectSolutions(
    std::map<int, int> robots,
    std::map<int, Target> targets,
    std::vector<PlacementOptimizer::Solution> &solutions,
    unsigned int mandatories,
    Solution partial
)
{
    if (robots.size()) {
        auto it = robots.begin();
        auto robot = it->second;
        robots.erase(it);
        
        for (auto &target : targets) {
            if (mandatories > robots.size() && !target.second.mandatory) {
                continue;
            }
            
            Solution solution = partial;
            solution.robotTarget[robot] = target.second;
            
            if (robots.size() == 0 || targets.size() == 0) {
                solutions.push_back(solution);
            } else {
                auto tmpTargets = targets;
                tmpTargets.erase(target.first);
                int tmpMandatory = mandatories;
                if (target.second.mandatory && tmpMandatory > 0) {
                    tmpMandatory -= 1;
                }
                
                collectSolutions(robots, tmpTargets, solutions, tmpMandatory, solution);
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
    unsigned int mandatories = 0;
    for (auto &target : targets) {
        if (target.mandatory) {
            mandatories++;
        }
        targetsMap[k++] = target;
    }
    collectSolutions(robotsMap, targetsMap, solutions, mandatories);
    
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