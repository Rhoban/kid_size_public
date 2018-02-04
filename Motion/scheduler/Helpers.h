#pragma once

#include <string>
#include <mutex>
#include <RhIO.hpp>
#include "services/Services.h"
#include "moves/Moves.h"

class MoveScheduler;

/**
 * Helpers
 *
 * Base class Helpers to low level
 * and scheduler access
 */
class Helpers
{
  public:

    /**
     * Initialization
     */
    Helpers();

    /**
     * Assign main MoveScheduler
     */
    void setScheduler(MoveScheduler* scheduler);

    /**
     * Getting the move scheduler
     */
    MoveScheduler* getScheduler();
    const MoveScheduler* getScheduler() const;

    /**
     * Getting services and moves
     */
    Services* getServices();
    const Services* getServices() const;
    Moves* getMoves();

    /**
     * Return true if lowlevel is in fake mode
     */
    bool isFakeMode() const;

    /**
     * Control moves
     */
    bool hasMove(const std::string& moveName) const;
    void startMove(const std::string& moveName, double fade=0.5) const;
    void stopMove(const std::string& moveName, double fade=0.5) const;

    /**
     * Helpers to access motors
     */
    virtual void setAngle(const std::string& servo, float angle);
    void setTorqueLimit(const std::string& servo, float torque);
    void setPGain(const std::string& servo, float gain);
    void setIGain(const std::string& servo, float gain);
    void setDGain(const std::string& servo, float gain);
    void setComplianceMarginCW(const std::string& servo, int gain);
    void setComplianceMarginCCW(const std::string& servo, int gain);
    void setComplianceSlopeCW(const std::string& servo, int gain);
    void setComplianceSlopeCCW(const std::string& servo, int gain);
    bool isRX(const std::string& servo);
    bool isMX(const std::string& servo);
    float getAngle(const std::string& servo);
    float getError(const std::string& servo);
    std::vector<std::string> getServoNames();

    /**
     * Accessing the IMU
     */
    float getPitch();
    float getRoll();
    float getYaw();
    float getGyroYaw();

    /**
     * Accessing the pressure sensor
     */
    float getPressureLeftRatio();
    float getPressureRightRatio();
    float getPressureWeight();

  private:

    /**
     * MoveScheduler main instance
     */
    MoveScheduler* _scheduler;
};
