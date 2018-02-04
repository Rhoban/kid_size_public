#pragma once

#include <mutex>
#include <Eigen/Dense>
#include <moves/Move.h>
#include <Spline/SmoothSpline.hpp>
#include <TrajectoryGeneration/TrajectoryUtils.h>

class ModelCalibration : public Move
{
    public:

        /**
         * Initialization
         */
        ModelCalibration();
        
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
         * Thread protection
         */
        std::mutex _mutex;

        /**
         * Iteration wait counter
         */
        int _countWait;

        /**
         * Save prefix path
         */
        std::string _logPrefix;

        /**
         * Current motion time 
         * and name.
         */
        std::string _motionName;
        double _motionTime;

        /**
         * Error message
         */
        std::string _errorMsg;

        /**
         * Motion experiment parameters
         */
        bool _motionIsSmooth;
        double _motionPeriod;
        double _motionAngleAmpl;
        double _motionCartAmpl;
        double _motionAngleStart;
        double _motionCartStart;
        std::string _motionTrajectory;
        Leph::Trajectories _trajs;

        /**
         * Generate either a step or a smooth trajectory
         * from start position with an amplitude
         * of ampl and a given time period length/
         * if isHalf is true, the movement only
         * goes from start to start+ampl.
         * Else, the movement goes from start, 
         * start+ampl and start-ampl and start.
         */
        Leph::Spline buildStepTraj(
            double start, double period, 
            double ampl, bool isHalf);
        Leph::Spline buildSmoothTraj(
            double start, double period, 
            double ampl, bool isHalf);

        /**
         * Start the calibration sequence
         * of given motion name
         */
        std::string cmdStart(std::string name);
};

