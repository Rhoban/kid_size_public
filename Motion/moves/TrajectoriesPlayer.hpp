#pragma once

#include <Eigen/Dense>
#include <moves/Move.h>
#include <Model/HumanoidFixedModel.hpp>
#include <TrajectoryGeneration/TrajectoryUtils.h>

class TrajectoriesPlayer : public Move
{
    public:

        /**
         * Initialization
         */
        TrajectoriesPlayer();
        
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
         * Humanoid model
         * (with given model parameters)
         */
        Leph::HumanoidFixedModel _model;
        
        /**
         * Trajectories filename
         */
        std::string _filename;

        /**
         * Model parameters path
         */
        std::string _modelParamsPath;

        /**
         * Splines container
         */
        Leph::Trajectories _trajs;

        /**
         * Is feedforward enabled
         */
        bool _isFeedForward;
        
        /**
         * Logs save path
         * (disable is empty)
         */
        bool _isLogging;
        std::string _logPath;

        /**
         * Play state
         */
        double _t;
        double _timeGain;
        bool _isContinue;

        /**
         * Trunk and foot 
         * position and orientation
         * offsets
         */
        Eigen::Vector3d _offsetTrunkPos;
        Eigen::Vector3d _offsetTrunkAxis;
        Eigen::Vector3d _offsetFootPos;
        Eigen::Vector3d _offsetFootAxis;

        /**
         * Reset and Start spline command
         */
        void cmdReset();
        std::string cmdStart();
};

