#pragma once

#include <string>
#include <vector>
#include <RhAL.hpp>
#include <time.h>
#include "services/Service.h"
#include <rhoban_utils/history/history.h>

class Move;

class MaintenanceService : public Service
{
    public:
        struct Task
        {
            std::string name;
            std::string description;
            time_t last;

            bool operator<(const Task&other) const {
                return other.last > last;
            }
        };

        MaintenanceService();

        void loadFile();
        void saveFile();
 
    protected:
        std::vector<Task> tasks;
        RhIO::Bind bind;

        std::string cmdMaintenance(std::string cmd);
};

