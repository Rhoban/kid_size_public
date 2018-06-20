#pragma once

#include <deque>
#include <map>
#include <string>
#include <rhoban_utils/spline/function.h>
#include "Move.h"
#include <Model/HumanoidFixedModel.hpp>
#include <Types/MatrixLabel.hpp>
#include <Utils/Scheduling.hpp>
#include <rhoban_unsorted/log_model.h>
#include <rhoban_utils/history/history.h>
#include <QuinticWalk/QuinticWalk.hpp>

class Kick;
class Walk : public Move
{
    public:
        Walk(Kick *kickMove);
        std::string getName();

        void onStart();
        void onStop();
        void updateParams(double factor, float step=0.0, float lateral=0.0, float turn=0.0);
        void step(float elapsed);

        /// Control the robot using [mm] and [deg]
        void control(bool enable, double step=0, double lateral=0, double turn=0);

        /**
         * Starts a kick
         * side: foot to use (0: left, 1: right)
         * name: type of kick to use
         */
        void kick(bool rightFoot, const std::string & kickName);
        
        /**
         * Are we kicking?
         */
        bool isKicking();

        /**
         * Are we moving?
         */
        bool isMoving();

        /**
         * Return true if a last kick 
         * is currently beeing done
         */
        bool isLastStep() const;

        /**
         * Return the current walk phase
         */
        double getPhase() const;

        /**
         * Return phase at last step
         */
        double getLastPhase() const;

        /**
         * Is the current step a new step?
         */
        bool isNewStep() const;

        /**
         * Return the current raw walk goal displacement
         * dX, dY, dTheta, enabledGain.
         */
        Eigen::Vector4d getRawOrder() const;

        /**
         * Return last command applied to walk (before trim)
         */
        Eigen::Vector4d getOrder() const;


        /**
         * Set raw (meters, radian) current walk
         * goal displacement: dX, dY, dTheta, enabled
         */
        void setRawOrder(
            double step, double lateral, 
            double turn, bool enable);
        /**
         * params: step[m], lat[m], rot [rad]
         */
        void setRawOrder(const Eigen::Vector3d & params, bool enabled);

        /**
         * Boundaries for orders and deltaOrders (step, lateral, turn)
         * units are: [m/step], [rad/step], [m/step^2] and [rad/step^2]
         */
        Eigen::Vector3d getMinOrders() const;
        Eigen::Vector3d getMaxOrders() const;
        Eigen::Vector3d getMinDeltaOrders() const;
        Eigen::Vector3d getMaxDeltaOrders() const;

        /**
         * If the walk is disable, ask for a last step of given
         * pose change. Step foot is automatically chosen.
         * [dx, dy, dtheta] in meters and radians.
         */
        void askLastStep(const Eigen::Vector3d& deltaPose);
        
        // Maximum rotation speed [deg/step]
        float maxRotation;

        // Maximum stepping [mm/step]
        float maxStep, maxStepBackward;

        // Maximum lateral [mm/step]
        float maxLateral;
        
        // Stops the walk (to debug the order received)
        bool dontWalk;
 
    protected:
        // Kick move
        Kick *kickMove;

        // Broadcast request to Kick
        //TODO: should it be moved to Kick?
        void updateShoot(bool left, const std::string & kickName);

        // Turn to false all flags relative to current shoot
        // If applyKick is true, then forward consequences of the kick to the
        // localisation service.
        void endShoot();
        
        // Kick warmup and cooldown
        double t;
        float waitT;
        float warmup;
        float cooldown;
        bool isWarmingUp;
        bool isCoolingDown;

        /**
         * External Walk Controls
         */
        bool lastWalkEnable;
        bool walkEnable;
        bool walkEnableTarget;
        float walkEnableTimeSinceChange;
        bool walkTransitionning;
        bool walkKickLeft;
        bool walkKickRight;
        float pressureY;
        float pressureYStd;
        float pressureYStdThresholdWarmup;
        float pressureYStdThresholdCooldown;
        rhoban_utils::History ratioHistory;
        std::string walkKickName;
        double walkStep;
        double walkLateral;
        double walkTurn;

        double phase, smoothing,timeSincePause;
        double smoothingStep, smoothingLateral, smoothingTurn;

        float startPhase;

        double swingRollGain,swingGain,swingPhase;
        double trunkXOffset_forward, trunkPitch_forward;
        double trunkXOffset_backward, trunkPitch_backward;

        double smoothCommands;
        double smoothTransition, P_stepPitch, P_stepXOffset, P_stepPitch_back, P_stepXOffset_back;
        double trunkYOffset;

        double compensateRoll;
        double leftRoll, rightRoll;

        /// Limits for delta orders [mm/step^2]
        double maxDStepByCycle, maxDLatByCycle, maxDTurnByCycle;

        /**
         * Values for kicking
         * TODO: which values are still used now that we have our own 'Kick' move?
         */
        bool prevKickLeft, prevKickRight;
        bool shouldKickLeft, shouldKickRight;
        bool shootingRight, shootingLeft;
        float shootT;
        float shootPhase;
        float shootAfterPhase;

        double trunkRoll, trunkRoll_pose;

        double securityPhase, securityTreshold;

        float xOffset, zOffset;

        Leph::QuinticWalk _engine;
        Eigen::Vector3d _orders;
        bool _isEnabled;
        Leph::VectorLabel _params;
        double _trunkZOffset;
        double _trunkHeight;
        double _footDistance;
        double _footYOffset;
        bool _securityEnabled;

        double _lastStepPhase;
        int _lastStepCount;
        Leph::VectorLabel _lastStepParams;

        Leph::Scheduling scheduling;

        float armsRoll, armsPitch;

        // Trimming
        float stepTrim, lateralTrim, turnTrim;

        // Model Logging
        rhoban_unsorted::LogModel _logs;

        // Apply walk params update at fixed phase
        double lastPhase;

	float elbowOffset;

	// GoalKeeper related attributs:
	bool gkMustRaise;
	bool gkMustBlock;
	float initElbowOffsetValue;
	float initArmsRollValue;
	float initTrunkZOffsetValue;
};
