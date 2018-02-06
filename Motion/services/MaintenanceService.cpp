#include "MaintenanceService.h"
#include <json/json.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/serialization/json_serializable.h>

using namespace rhoban_utils;

static rhoban_utils::Logger logger("Maintenance");

MaintenanceService::MaintenanceService()
    : bind("maintenance")
{
    bind.bindFunc("maintenance", "Maintenance operations",
            &MaintenanceService::cmdMaintenance, *this, {"list"});

    loadFile();
}

void MaintenanceService::loadFile()
{
    tasks.clear();

    Json::Value json;
    try {
      json = rhoban_utils::file2Json("maintenance.json");
    } catch (const rhoban_utils::JsonParsingError & exc) {
      logger.error("%s", exc.what());
      return;
    }

    for (auto name : json.getMemberNames()) {
      auto entry = json[name];
      
      Task task;
      task.name = name;
      task.description = entry["description"].asString();
      task.last = entry["last"].asInt();
      tasks.push_back(task);
    }
}

void MaintenanceService::saveFile()
{
    Json::Value json;
        
    for (auto &task : tasks) {
        Json::Value entry;
        entry["description"] = task.description;
        entry["last"] = (int)task.last;
        json[task.name] = entry;
    }

    Json::StyledWriter writer;
    std::string data = writer.write(json);

    file_put_contents("maintenance.json", data);
}

static std::string prettyTime(int elapsed)
{
    std::stringstream ss;

    if (elapsed < 60) {
        ss << elapsed << " secs ago";
    } else if (elapsed < 3600) {
        ss << (elapsed/60) << " mins ago";
    } else if (elapsed < 3600*24) {
        ss << (elapsed/3600) << " hrs ago";
    } else if (elapsed < 3600*24*100) {
        int h = (elapsed%(3600*24))/3600;
        int d = (elapsed-h*3600)/(3600*24);
        ss << d << " days, " << h << " hrs ago";
    } else {
        ss << "A long time ago";
    }

    return ss.str();
}
        
std::string MaintenanceService::cmdMaintenance(std::string cmd)
{
    std::stringstream ss;

    if (cmd != "list") {
        bool ok = false;
        for (auto &task : tasks) {
            if (task.name == cmd) {
                task.last = time(NULL);
                ok = true;
            }
        }

        if (ok) {
            saveFile();
            ss << "Task " << cmd << " marked as updated, good job!";
            return ss.str();
        } else {
            return "ERROR: Can't find given task";
        }
    }
    
    char buffer[256];
    sprintf(buffer, "%-20s %-20s %s", "Last update", "Task name", "Task description");
    ss << buffer << std::endl;;
    sprintf(buffer, "%-20s %-20s %s", "---------", "---------", "---------");
    ss << buffer << std::endl;
    std::sort(tasks.begin(), tasks.end());
    for (auto task : tasks) {
        double elapsed = (time(NULL)-task.last);
        auto t = prettyTime(elapsed);

        sprintf(buffer, "%-20s %-20s %s", t.c_str(), task.name.c_str(), task.description.c_str());

        ss << buffer << std::endl;;
    }
    ss << std::endl;
    ss << "To mark a maintenance task as done, run: maintenance [task_name]";

    return ss.str();
}
