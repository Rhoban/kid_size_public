#pragma once

#include <vector>
#include <map>
#include <string>

class Move;
class MoveScheduler;

/**
 * Moves
 *
 * Moves container
 */
class Moves
{
    public:

        /**
         * Initialization of all moves
         * with main MoveScheduler instance
         */
        Moves(MoveScheduler* scheduler);

        /**
         * Deallocation of all moves
         */
        ~Moves();

        /**
         * Moves access
         */
        bool hasMove(const std::string& name) const;
        Move* getMove(const std::string& name);
        const Move* getMove(const std::string& name) const;
        const std::vector<std::pair<std::string, Move*>>& getAllMoves();

    private:
        
        /**
         * Main MoveScheduler pointer
         */
        MoveScheduler* _scheduler;

        /**
         * Moves container and map
         * (For direct access and well defined iteration order).
         */
        std::vector<std::pair<std::string, Move*>> _container;
        std::map<std::string, Move*> _map;

        /**
         * Insert a new given Move
         */
        void add(Move* move);

};

