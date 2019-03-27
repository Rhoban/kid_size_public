#pragma once

#include "services/Service.h"

#include <vive_provider/udp_message_manager.h>

#include <Eigen/Geometry>

/**
 * Provide interface to the data provided by rhoban vive provider
 */
class ViveService : public Service
{
public:
  ViveService();

  /**
   * Implement ElapseTick.
   */
  bool tick(double elapsed) override;

  /**
   * Return true if the current configuration of the vive is valid and messages were received
   */
  bool isActive() const;

  /**
   * Return the transformation from the field basis to the vive basis.
   * time_stamp is in micro-seconds and can be specified according to local steady_clock or according to system_clock.
   * throws an out_of_range error if status does not contain information for the tracker id
   */
  Eigen::Affine3d getFieldToVive(uint64_t time_stamp, bool system_clock = false) const;

  /**
   * @see getFieldToVive
   */
  Eigen::Affine3d getFieldToCamera(uint64_t time_stamp, bool system_clock = false) const;

  /**
   * Return the transformation from vive referential to camera referential
   */
  Eigen::Affine3d getViveToCamera() const;

  void setPosOffset(const Eigen::Vector3d& pos);
  void setRoll(double roll);
  void setPitch(double pitch);
  void setYaw(double yaw);

  std::string cmdVive();

private:
  /**
   * Manager for vive messages
   */
  vive_provider::UDPMessageManager vive_manager;

  /**
   * RhIO binding
   */
  RhIO::Bind bind;

  double camera_x;
  double camera_y;
  double camera_z;

  double camera_roll;
  double camera_pitch;
  double camera_yaw;

  std::string tracker_serial;
};
