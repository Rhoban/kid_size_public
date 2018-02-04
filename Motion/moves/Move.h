#pragma once

#include <string>
#include <RhIO.hpp>
#include "scheduler/Helpers.h"
#include <rhoban_utils/timing/elapse_tick.h>

class MoveScheduler;

class Move : public Helpers, public rhoban_utils::ElapseTick
{
    public:

        /**
         * Initialization
         */
        Move();

        /**
         * Virtual destructor
         */
        virtual ~Move();

        /**
         * Return the name of the move.
         * Have to be implemented.
         */
        virtual std::string getName() = 0;

        /**
         * Main Move implementation.
         * Elapsed is time duration between last 
         * call in seconds.
         */
        virtual void step(float elapsed) = 0;

        /**
         * Optional on start ans on stop
         * move callback
         */
        virtual void onStart();
        virtual void onStop();

        /**
         * Return current move state.
         * If true, the move is ticked.
         */
        bool isRunning() const;
        bool isFading() const;

        /**
         * Getting current move 
         * status and errors strings
         */
        virtual std::string getStatus() const;
        std::string getError() const;

        /**
         * Control the move
         */
        void start(double fade=-1);
        void stop(double fade=-1);

        /**
         * Implement ElapseTick tick
         */
        virtual bool tick(double elapsed) override final;
        
        /**
         * Setting angle (forwards to the helpers applying smoothing)
         */
        void setAngle(const std::string& servo, float angle) override;

        /**
         * Initialize the RhIO binding
         * fo this Move
         * (Should not be called by the user).
         */
        void initializeBinding();

    protected:
        
        /**
         * RhIO binding
         */
        RhIO::Bind* bind;

        /**
         * Smoothing gain
         */
        enum { SMOOTHING_IN, SMOOTHING_RUNNING, SMOOTHING_OUT } _smoothingState;
        double _smoothingRemaining, _smoothingDuration;
        double _smoothing;
        
    private:
        
        /**
         * Move running state
         */
        bool _isRunning;

        /**
         * Last moves error message (exception)
         */
        std::string _error;
        
        /**
         * Start RhIO command
         */
        std::string cmdStart();
};

