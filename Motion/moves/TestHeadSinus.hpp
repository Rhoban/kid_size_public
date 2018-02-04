#pragma once

#include "Move.h"

class TestHeadSinus : public Move
{
    public:
        
        /**
         * Initialization
         */
        TestHeadSinus();

        /**
         * Implement Move
         */
        virtual std::string getName() override;
        virtual void onStart() override;
        virtual void step(float elapsed) override;

    private:

        /**
         * Affix movement
         */
        double _t;

        /**
         * Frequency
         */
        double _freqTilt;
        double _freqPan;

        /**
         * Amplitude
         */
        double _amplTilt;
        double _amplPan;
};

