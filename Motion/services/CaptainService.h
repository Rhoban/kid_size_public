#pragma once

#include <rhoban_geometry/point.h>
#include <rhoban_utils/serialization/json_serializable.h>
#include <rhoban_team_play/team_play.h>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include <strategy/PlacementOptimizer.h>
#include <thread>
#include "Service.h"

class ApproachMove;
class KickController;

class CaptainService : public Service
{
public:
    struct Instruction
    {
        rhoban_geometry::Point targetPosition;
        float targetOrientation;
    };
    
    struct BasePosition
    {
        rhoban_geometry::Point targetPosition;
        bool mandatory;
        bool kickOff;
        float targetOrientation;
    };
    
    struct Config : public rhoban_utils::JsonSerializable
    {
        std::string getClassName() const;
        void fromJson(const Json::Value & json_value, const std::string & dir_name);
        Json::Value toJson() const;
    
        std::vector<BasePosition> basePositions;
    };
    
    CaptainService();
    virtual ~CaptainService();
    
    bool tick(double elapsed) override;
    
    int captainId();
    bool IAmCaptain();
    
    rhoban_team_play::CaptainInfo getInfo();
    
    Instruction getInstruction();
    
protected:
    Config config;
    float frequency;
    RhIO::Bind bind;
    
    volatile bool running;
    std::thread *captainThread;
    std::mutex mutex;
    rhoban_team_play::CaptainInfo info;
    
    /**
    * Computation of the captain orders
    */
    void compute();
    
    /**
    * Execution thread
    */
    void execThread();
    
    /**
     * Setting a placement optimizer solution
     */
    void setSolution(PlacementOptimizer::Solution solution);
    
    /**
    * UDPBroadcast instance
    */
    rhoban_utils::UDPBroadcast* _broadcaster;
};
