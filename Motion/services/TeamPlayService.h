#pragma once

#include <string>
#include <map>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include <rhoban_team_play/team_play.h>
#include <hl_communication/udp_message_manager.h>
#include <hl_communication/wrapper.pb.h>
#include <RhIO.hpp>
#include "services/Service.h"

/**
 * Convert a rhoban_team_play info to GameMsg (possible loss)
 */
void exportTeamPlayToGameWrapper(const rhoban_team_play::TeamPlayInfo & myInfo,
                                 int team_id,
                                 hl_communication::GameMsg * dst);
/**
 * TeamPlay
 *
 * UDP broadcast playing and robot state 
 * to others players and monitoring
 */
class TeamPlayService : public Service
{
    public:

        /**
         * Initialization
         */
        TeamPlayService();

        /**
         * Destructor
         */
        virtual ~TeamPlayService();

        /**
         * Return the robot teamplay id
         */
        int myId();
        
        /**
         * Access a teamplay info 
         * struct of current robot
         */
        const rhoban_team_play::TeamPlayInfo& selfInfo() const;
        rhoban_team_play::TeamPlayInfo& selfInfo();

        /**
         * Access to container to information 
         * on available other players
         */
        const std::map<int, rhoban_team_play::TeamPlayInfo>& allInfo() const;

        /**
         * Read/Write network and 
         * update outputs
         */
        bool tick(double elapsed);

        /**
         * Is team play enabled ?
         */
        bool isEnabled();
    
 
        /**
         * TeamPlay radius
         */
        double teamRadius;

        /**
         * Extra radius for the referee
         */
        double refereeRadius;
        
    private:

        /**
         * RhIO Binding
         */
        RhIO::Bind* _bind;

        /**
         * UDPBroadcast instance
         */
        rhoban_utils::UDPBroadcast* _broadcaster;

        /**
         * Protobuf message manager
         */
        std::unique_ptr<hl_communication::UDPMessageManager> _protobuf_message_manager;

        /**
         * Current and other robots infos
         */
        rhoban_team_play::TeamPlayInfo _selfInfo;
        std::map<int, rhoban_team_play::TeamPlayInfo> _allInfo;

        /**
         * Protobuf Communication
         */
        hl_communication::GameMsg _myMessage;

        /**
         * Send and check 
         * delay in seconds
         */
        double _t;
        
        /**
         * Enable or disable the teamplay
         */
        bool _isEnabled;

        /**
         * Broadcast period in seconds
         */
        double _broadcastPeriod;

        /**
         * Do message sending if needed
         */
        void messageSend();

        /**
         * Parse given message 
         * and update teamplay state
         */
        void processInfo(rhoban_team_play::TeamPlayInfo info);

        /**
         * Is the placing aggressive or defensive?
         */
        double aggressivity;
        
        /**
         * RhIO team command
         */
        std::string cmdTeam();

};

