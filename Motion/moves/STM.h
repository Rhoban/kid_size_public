#pragma once

#include "Move.h"

class STM : public Move
{
    public:
        std::string getStatus() const;

        virtual void setState(std::string state);
        virtual void enterState(std::string state);
        virtual void exitState(std::string state);

    protected:
        std::string state;
        std::string nextState;
};
