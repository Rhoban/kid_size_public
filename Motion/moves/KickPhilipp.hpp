#pragma once

#include <Eigen/Dense>
#include <moves/Move.h>
#include <TrajectoryGeneration/TrajectoryUtils.h>

class KickPhilipp : public Move
{
    public:

        /**
         * Initialization
         */
        KickPhilipp();
        
        /**
         * @Inherit
         */
        virtual std::string getName() override;

        /**
         * @Inherit
         */
        virtual void onStart() override;
        
        /**
         * @Inherit
         */
        virtual void step(float elapsed) override;

    private:
        
        /**
         * Trajectories filename
         */
        std::string _filename;

        /**
         * Splines container
         */
        Leph::Trajectories _trajs;

        /**
         * Play state
         */
        double _t;
        double _timeGain;

        /**
         * Reset and Start spline command
         */
        void cmdReset();
        std::string cmdStart();
};

