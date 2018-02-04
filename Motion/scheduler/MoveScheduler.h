#pragma once

#include <string>
#include <RhIO.hpp>
#include <RhAL.hpp>
#include <Bindings/RhIOBinding.hpp>

#include "moves/Moves.h"
#include "moves/Move.h"
#include "services/Services.h"
#include "services/Service.h"

class Move;
class Services;

/**
 * MoveScheduler
 *
 * Main Motion class for Moves, Services 
 * and low level management
 */
class MoveScheduler
{
    public:

        /**
         * Initialization
         */
        MoveScheduler();

        /**
         * Destructor.
         * Deallocate all services and moves.
         */
        ~MoveScheduler();

        /**
         * Moves state management
         */
        bool hasMove(const std::string& name);
        void startMove(const std::string& name, double fade);
        void stopMove(const std::string& name, double fade);
        void stopAllMoves(double fade);

        /**
         * Emergency.
         * Disable all servos.
         */
        void releaseServos();

        /**
         * Runs the move scheduler
         * Does not return until askQuit() is called.
         * Main motion application loop.
         */
        void execute();

        /**
         * Access to all services
         */
        Services* getServices();
        const Services* getServices() const;

        /**
         * Return true if MoveScheduler is using a fake protocol
         */
        bool isFakeMode();
        
        /**
         * Access to all moves
         */
        Moves* getMoves();
        
        /**
         * Access to given move
         */
        Move* getMove(const std::string& name);
        const Move* getMove(const std::string& name) const;

        /**
         * Access to low level RhAL Manager
         */
        RhAL::StandardManager* getManager();

        /**
         * Ask for MoveScheduler 
         * exit of execute()
         */
        void askQuit();

    private:

        /**
         * Main loop execute() state
         */
        bool _isOver;

        /**
         * Main loop statictics
         */
        double _minTimeLoop;
        double _avgTimeLoop;
        double _maxTimeLoop;
        double _minTimeFlush;
        double _avgTimeFlush;
        double _maxTimeFlush;
        double _minTimeTickMoves;
        double _avgTimeTickMoves;
        double _maxTimeTickMoves;
        double _minTimeTickServices;
        double _avgTimeTickServices;
        double _maxTimeTickServices;

        /**
         * RhAL low level manager
         */
        RhAL::StandardManager _manager;

        /**
         * Services container
         */
        Services* _services;
        
        /**
         * Moves container
         */
        Moves* _moves;

        /**
         * RhAL RhIO Binding
         */
        RhAL::RhIOBinding* _binding;

        /**
         * RhIO Binding
         */
        RhIO::Bind* _bind;

        /**
         * RhIO commands
         */
        std::string cmdMoves();
        std::string cmdStart(std::string name, double fade);
        std::string cmdInit();
        std::string cmdStop(std::string name, double fade);
        std::string cmdEm();
        std::string cmdTare();
        std::string cmdGyroTare();
        std::string cmdStats();
};
