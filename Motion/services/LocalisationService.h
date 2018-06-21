#pragma once

#include <RhIO.hpp>
#include "Service.h"

#include <Eigen/Dense>
#include <rhoban_geometry/point.h>
#include <rhoban_utils/timing/time_stamp.h>

namespace Vision {
class Robocup;
class LocalisationBinding;
}

/// Unless explicitely stated otherwise, units are SI
class LocalisationService : public Service
{
  
    public:
        LocalisationService();

        // Ball
        rhoban_geometry::Point getBallPosSelf();
        rhoban_geometry::Point getBallPosWorld();
        rhoban_geometry::Point getBallPosField();
        rhoban_geometry::Point getBallSpeedSelf();
        rhoban_geometry::Point getPredictedBallSelf();
        rhoban_geometry::Point getPredictedBallSelf(rhoban_utils::TimeStamp t);
        rhoban_geometry::Point getLookBallPosWorld();
        Eigen::Vector3d ballPosWorld;
        Eigen::Vector3d ballLookPosWorld;
        float ballQ;
        rhoban_geometry::Point ballSpeed;// In world referential
        rhoban_utils::TimeStamp ballTS;
        
        // Goal
        rhoban_geometry::Point getGoalPosWorld();
        rhoban_geometry::Point getGoalPosSelf();
        rhoban_geometry::Point getGoalPosField();
        rhoban_geometry::Point getOurGoalPosField();
        rhoban_geometry::Point getLeftGoalPosSelf();
        rhoban_geometry::Point getRightGoalPosSelf();
        rhoban_geometry::Point getFieldPos();
        double getFieldOrientation();// Return value in [rad]
        Eigen::Vector3d goalLeftPosWorld;
        Eigen::Vector3d goalRightPosWorld;
        Eigen::Vector3d fieldCenterWorld;
        rhoban_utils::Angle getOurBallToGoalDirSelf();

        float fieldQ, fieldConsistency;
        bool consistencyEnabled;
        double getGoalCap();
        double getLeftGoalCap();
        double getRightGoalCap();
        double getPenaltyLeftGoalCap();
        double getPenaltyRightGoalCap();
        rhoban_geometry::Point getGoalPos();

        // Opponent
        std::vector<rhoban_geometry::Point> getOpponentsField();
        double opponentsRadius;
        bool opponentsAreFake;
        std::vector<Eigen::Vector3d> opponentsWorld;
        void updateOpponentsPos();
        void setOpponentsWorld(const std::vector<Eigen::Vector3d> &pos);

        /// Update the string representing shared_opponents
        void updateSharedOpponentsPos();

       /// Return a thread safe copy of the team mates positions by Id
       std::map<int, Eigen::Vector3d> getTeamMatesField();

       /// Return the list of opponents seen by other mates
       std::vector<Eigen::Vector2d>  getSharedOpponents();

       /// robot_id is the robot providing the opponent Positions
       void updateSharedOpponents(int robot_id,
                                  const std::vector<Eigen::Vector2d> & opponentPos);

       /// Remove all obstacles provided by this robot
       void removeSharedOpponentProvider(int id);

       /// Occupation of a teamMate for obstacle avoidance [m]
       double teamMatesRadius;

       /// Stored position of the team mates currently playing
       /// values: (x[m], y[m], dir[rad])
       std::map<int, Eigen::Vector3d>  teamMatesField;

       /// Key: id of robot providing the information
       /// Value: List of opponents currently seen by the robot
       std::map<int, std::vector<Eigen::Vector2d>>  sharedOpponentsField;

        // Tell the localisation service we see the ball
        void setBallWorld(const Eigen::Vector3d &pos,
                          const Eigen::Vector3d &lookPos,
                          float quality,
                          const rhoban_geometry::Point & speed,
                          const rhoban_utils::TimeStamp & ts);
        void updateBallPos();
        void setNoBall();
        
        // Tell the localisation service we know where the goals are
        void setPosSelf(const Eigen::Vector3d &left, 
			const Eigen::Vector3d &right, 
			const Eigen::Vector3d &center, float orientation, 
                        float quality, float consistency, bool consistencyEnabled=false);
        void updatePosSelf();

        // Goal Scanner target
        float goalTargetPan;
        float goalTargetTilt;

        /// x,y -> [m]
        void applyKick(float x, float y);

        // How many frames were captured by the camera?
        int getFrames();

        // Accessing vision status
        std::string getCameraStatus();
        double getLastVisionUpdate();
        bool isVisionActive() const;

        void resetBallFilter();
        void resetFieldFilter();
        void resetRobotFilter();

        void gameStartReset();
        void kickOffReset();
        void dropBallReset();
        /// x in [m]
        void penaltyReset(float x);
        void penaltyGoalReset();
        void goalReset();

        /// x,y, noise in meters, theta thetaNoise in deg
        void customFieldReset(double x, double y, double noise, double theta, double thetaNoise);
        void fallReset();
        void bordersReset();

        void setVisualCompassStatus(bool inUse);
        bool getVisualCompassStatus() const;

        rhoban_geometry::Point worldToField(Eigen::Vector3d world);

        void enableFieldFilter(bool enable=true);
	void isGoalKeeper(bool status=false);

#ifdef VISION_COMPONENT
	void stealTags(std::vector<int> & indices,
                 std::vector<Eigen::Vector3d> & positions,
                 std::vector<std::pair<float, float> > & centers,
                 std::vector<std::pair<float, float> > & centersUndistorded,
                 double * timestamp);
#endif
	
#ifdef VISION_COMPONENT
        void setRobocup(Vision::Robocup *robocup);
        void setLocBinding(Vision::LocalisationBinding * locBinding);
#endif
        
    protected:
        // Mutex to access ball position
        std::mutex mutex;
        
        // For RhIO
        float ballPosX, ballPosY;
        float ballFieldX, ballFieldY;
        // Informations provided by the speed estimator
        float ballSpeedXInSelf, ballSpeedYInSelf;
        float ballPredictedX, ballPredictedY;
        
        // For RhIO
        float goalPosX, goalPosY, goalCap, goalLeftCap, goalRightCap;
        float fieldPosX, fieldPosY, fieldOrientation, fieldOrientationWorld;
        std::string opponents;
        std::string mates;
        std::string sharedOpponents;
        
        // Ticking
        bool tick(double elapsed) override;

        RhIO::Bind bind;
        // Used for warning the user on frequent kicks
        ::rhoban_utils::TimeStamp lastKick;

#ifdef VISION_COMPONENT
        Vision::Robocup *robocup;
        Vision::LocalisationBinding * locBinding;
#endif

    private:

        bool simulateWalk;

        /// Is the visual compass currently enabled in Localisation?
        bool visualCompassActivated;

        /**
         * RhIO command to set the ball /goal position in fake mode
         */
        std::string cmdFakeBall(double x, double y);
        std::string cmdFakeOpponents(std::vector<std::string> args);
        std::string cmdFakeLoc(
            double leftX, double leftY, 
            double rightX, double rightY, 
            double fieldX, double fieldY);
        std::string cmdResetPosition();
        std::string cmdMoveOnField(
                double x, double y, double yaw
                );
};
