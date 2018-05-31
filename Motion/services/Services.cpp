#include <stdexcept>
#include "services/Services.h"
#include "services/Service.h"

#include "ModelService.h"
#include "RefereeService.h"
#include "LocalisationService.h"
#include "TeamPlayService.h"
#include "DecisionService.h"
#include "MaintenanceService.h"
#include "StrategyService.h"
#include "CaptainService.h"

Services::Services(MoveScheduler* scheduler) : 
    _scheduler(scheduler)
{
    if (_scheduler == nullptr) {
        throw std::logic_error("Services null pointer");
    }

    //Services instanciation
    model = new ModelService;
    add("model", model);
    referee = new RefereeService;
    add("referee", referee);
    localisation = new LocalisationService;
    add("localisation", localisation);
    teamPlay = new TeamPlayService;
    add("teamPlay", teamPlay);
    decision = new DecisionService;
    add("decision", decision);
    maintenance = new MaintenanceService;
    add("maintenance", maintenance);
    strategy = new StrategyService;
    add("strategy", strategy);
    captain = new CaptainService;
    add("captain", captain);
}

Services::~Services()
{
    for (const auto& it : _container) {
        delete it.second;
    }
}

bool Services::hasService(const std::string& name) const
{
    return (_map.count(name) > 0);
}
Service* Services::getService(const std::string& name)
{
    if (!hasService(name)) {
        throw std::logic_error(
            "Services service does not exist: " + name);
    } else {
        return _map.at(name);
    }
}
const std::vector<std::pair<std::string, Service*>>& Services::getAllServices()
{
    return _container;
}

void Services::add(const std::string& name, Service* service)
{
    service->setScheduler(_scheduler);
    _container.push_back({name, service});
    _map.insert({name, service});
}

