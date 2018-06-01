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
    /**
     * Captain instruction for a robot
     */
    struct Instruction
    {
        // Target position & orientation
        rhoban_geometry::Point targetPosition;
        float targetOrientation;
        
        // Captain order
        rhoban_team_play::CaptainOrder order;
    };
    
    /**
     * Captain configuration
     */
    struct Config : public rhoban_utils::JsonSerializable
    {
        struct BasePosition
        {
            rhoban_geometry::Point targetPosition;
            bool mandatory;
            bool kickOff;
            float targetOrientation;
        };
        
        std::string getClassName() const;
        void fromJson(const Json::Value & json_value, const std::string & dir_name);
        Json::Value toJson() const;
    
        std::vector<BasePosition> basePositions;
    };
    
    CaptainService();
    virtual ~CaptainService();
    
    bool tick(double elapsed) override;
    
    /**
     * Returns the captain ID
     */
    int captainId();
    
    /**
     * Am I the captain ?
     */
    bool IAmCaptain();
    
    /**
     * Getting captain info
     */
    rhoban_team_play::CaptainInfo getInfo();
    
    /**
     * Get the captain instruction for us (see above)
     */
    Instruction getInstruction();
    
protected:
    // RhIO binding
    RhIO::Bind bind;
    
    // Captain configuration
    Config config;
    float frequency;
    
    // Captain asynchronous thread
    volatile bool running;
    std::thread *captainThread;
    std::mutex mutex;
    
    // Either the info computed or the one grabbed from the captain
    rhoban_team_play::CaptainInfo info;
    
    // Available robots
    std::map<int, rhoban_team_play::TeamPlayInfo> robots;
    std::vector<int> robotIds;
    
    // Geometric parameters
    double passPlacingRatio, passPlacingOffset;
    double perpendicularBallDistance;
    double placingBallDistance;
    double avoidRadius;
    double aggressivity;
    
    std::vector<PlacementOptimizer::Target> getTargetPositions(rhoban_geometry::Point ball,
        rhoban_geometry::Point ballTarget);
    
    /**
     * Do the update for the base positions
     */
    void computeBasePositions();
    
    /**
     * Do the update for the game position
     */
    void computePlayingPositions();
    
    /**
    * Computation of the captain orders
    */
    void compute();
    
    /**
    * Execution thread
    */
    void execThread();
    
    /**
     * Updates the captain info from a placement optimizer solution
     */
    void setSolution(PlacementOptimizer::Solution solution);
    
    /**
    * UDPBroadcast instance
    */
    rhoban_utils::UDPBroadcast* _broadcaster;
};
