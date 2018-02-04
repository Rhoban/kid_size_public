#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <Utils/RandomWalk.hpp>
#include <Eigen/Dense>
#include <Model/HumanoidFixedModel.hpp>
#include "Move.h"

#include <Eigen/StdVector>

class Walk;
class OdometryCalibration : public Move
{
    public:
        
        /**
         * Initialization
         */
        OdometryCalibration(Walk* walk);

        /**
         * Implement Move
         */
        virtual std::string getName() override;
        virtual void onStart() override;
        virtual void step(float elapsed) override;

    private:
        
        /**
         * Walk instance
         */ 
        Walk* _walk;
        
        /**
         * RhIO user message for 
         * calibration state
         */
        std::string _message;

        /**
         * Measured target displacement.
         * Target dispacement is z (rotation)
         * is between [0:12] in "hours" format.
         */
        Eigen::Vector3d _displacement;

        /**
         * If true, odometry pose in world frame
         * is recorded in container for read 
         * and goal Model.
         */
        bool _isLogging;

        /**
         * Read and Goal Model planar pose trajectories
         * and support foot along learning data 
         * experiments in world frame.
         * Walk orders and phase.
         */
        std::vector<std::vector<
            Eigen::Vector3d>> _readTrajsPose;
        std::vector<std::vector<
            Leph::HumanoidFixedModel::SupportFoot>> _readTrajsSupport;
        std::vector<std::vector<
            Eigen::Vector3d>> _goalTrajsPose;
        std::vector<std::vector<
            Leph::HumanoidFixedModel::SupportFoot>> _goalTrajsSupport;
        std::vector<std::vector<
            Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>> _walkTrajsOrder;
        std::vector<std::vector<
            double>> _walkTrajsPhase;
        
        /**
         * Target current displacement for associated sequence
         * Target dispacement is z (rotation)
         * is between [0:12] in "hours" format.
         */
        std::vector<Eigen::Vector3d> _targetDisplacements;
        
        /**
         * Protection mutex
         */
        std::mutex _mutex;

        /**
         * Odometry dump file.
         * Dump is enabled if not empty
         */
        std::string _dumpFileName;

        /**
         * Automatic walk control
         * during calibration
         */
        bool _isAutoWalk;
        bool _autoEnable;
        double _autoMinStep;
        double _autoMaxStep;
        double _autoMinLateral;
        double _autoMaxLateral;
        double _autoMinTurn;
        double _autoMaxTurn;
        double _autoDeltaStep;
        double _autoDeltaLateral;
        double _autoDeltaTurn;
        double _autoInertia;
        double _autoLastPhase;
        Leph::RandomWalk _randomWalk;

        /**
         * If > 0, then represent the number of cycles used for autoWalk
         */
        int _autoNbCycles;
        /**
         * Number of cycles elapsed in auto
         */
        int _cycleCounts;

        /**
         * Set reference relative displacement 
         * in source position frame.
         * If setLast is true, assign 
         * last recorded sequence
         * displacement with given one
         */
        std::string cmdReferenceDisplacement(
            double dx, double dy, double da, bool setLast);

        /**
         * Start, stop or cancel logging a new 
         * learning odometry sequence
         */
        std::string cmdStartLog();
        std::string cmdStopLog();
        std::string cmdCancelLog();
        
        /**
         * Request calibration data dump
         */
        std::string cmdDumpData(std::string filename);
};

