#pragma once

#include <rhoban_geometry/point.h>
#include <deque>
#include <map>

class PlacementOptimizer
{
public:
    struct Target
    {
        rhoban_geometry::Point position;
        float orientation;
        float data;
    };
    
    struct Solution
    {
        std::map<int, Target> robotTarget;
    };
    
    static void collectSolutions(
        std::map<int, int> robots,
        std::map<int, Target> targets,
        std::vector<Solution> &solutions,
        Solution partial = Solution()
    );
    
    static Solution optimize(
        std::vector<int> robots,
        std::vector<Target> targets,
        std::function<float (Solution)> scoreFunc
    );
};