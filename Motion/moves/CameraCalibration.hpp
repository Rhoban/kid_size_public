#pragma once

#include <vector>
#include <string>
#include <map>
#include <random>
#include <Model/HumanoidModel.hpp>
#include <Eigen/Dense>
#include <Types/MatrixLabel.hpp>
#include "Move.h"

/**
 * Record data for offline
 * camera aperture and model
 * calibration
 */
class CameraCalibration : public Move
{
    public:
        
        /**
         * Initialization
         */
        CameraCalibration();

        /**
         * Implement Move
         */
        virtual std::string getName() override;
        virtual void onStart() override;
        virtual void step(float elapsed) override;

    private:

        /**
         * Random generator
         */
        std::default_random_engine _generator;
        
        /**
         * Container for calibration data
         */
        Leph::MatrixLabel _container;    

        /**
         * Current target view point on ground
         * and target pixel on camera space
         */
        Eigen::Vector2d _pixelPoint;
        Eigen::Vector3d _groundPoint;

        /**
         * Output data path
         */
        std::string _outputDataFile;

        /**
         * Index in calibration process
         */
        size_t _calibIndex;

        /**
         * The number of samples per
         * ground target distance
         */
        int _samplesNumber;

        /**
         * RhIO user message for 
         * calibration state
         */
        std::string _message;

        /**
         * Generate a new target point.
         * Return false is the calibration
         * is finished.
         */
        bool setNextPoint();

        /**
         * Reset and start a new 
         * calibration process
         */
        std::string cmdStartCalib();

        /**
         * Save current state and 
         * chose new target
         */
        std::string cmdNextPoint();

        /**
         * Generate a new pixel target
         * in case of unreachable point
         */
        void cmdRenewPoint();

        /**
         * Estimate the ground position
         * of current point at given
         * pixel space position
         */
        std::string cmdTestPoint(double pixelX, double pixelY);
};

