#pragma once

#include <vector>
#include "Service.h"
#include <robocup_referee/referee_client.h>

namespace RhIO
{
    class Bind;
}

class RefereeService : public Service, public robocup_referee::RefereeClient
{
    public:
        RefereeService();
        ~RefereeService();

        /**
         * Updating the playing flag
         */
        void checkPlaying();

        /**
         * Am I playing ?
         * (This is false when penalized)
         */
        bool isPlaying();

        /**
         * Is the game playing ?
         * (This is true even when penalized)
         */
        bool isGamePlaying();

        /// This phase happens only at the beginning of the game
        bool isInitialPhase();

        /// During this phase robots are allowed to move to their own field
        bool isPlacingPhase();

        /// During this phase robot should freeze
        bool isFreezePhase();

        /**
         * Is it the begining of the game?
         */
        bool isBegining();

        /**
         * Time since playing (reseted if penalized)
         */
        double getTimeSincePlaying();

        /**
         * Should we avoid touching the ball, regarding the rules annd the
         * current referee state?
         */
        bool shouldLetPlay();

        /**
         * Am I penalized?
         */
        bool isPenalized();

        /**
         * Is the given player penalized?
         */
        bool isPenalized(int id);

        /**
         * Is our team taking the kickoff?
         */
        bool myTeamKickOff();

        /**
         * Are we in a dropped ball scenario ?
         */
        bool isDroppedBall();

        /**
         * Is there a free kick right now?
         */
        bool isFreeKick();

        /**
         * Is the free kick for our team?
         */
        bool myTeamFreeKick();

        /**
         * Secondary time
         */
        int getSecondaryTime();

        void resetTimer();

        int gameTime();

        bool tick(double elapsed) override;
        double timeSincePlaying;
        double timeSinceGamePlaying;
        int remaining;

        int id, teamId;
        int alive;
        bool force, playing, gamePlaying;

        bool wasPenalized;

        bool dumpGameState;

    protected:
        RhIO::Bind *bind;
        float beginDuration;

        /**
         * Textual referee state
         */
        std::string _state;

        std::string cmdPlaying();

        void setTextualState();
};
