#ifndef INDOORUAV_DJI_DRIVER_MAVIC_3T_GIMBAL_CONTROLLER_H_
#define INDOORUAV_DJI_DRIVER_MAVIC_3T_GIMBAL_CONTROLLER_H_

#include <ros/ros.h>

#include <dji_gimbal_manager.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include "indooruav_dji_driver/PSDK_GimbalPitchAngleInDeg.h"

class DjiGimbalController {
 public:
  explicit DjiGimbalController(T_DjiOsalHandler* osal_handler);
  DjiGimbalController() = delete;
  DjiGimbalController(const DjiGimbalController& other) = delete;
  DjiGimbalController& operator=(const DjiGimbalController& other) = delete;
  ~DjiGimbalController();

 private:
  T_DjiReturnCode Init();
  T_DjiReturnCode DeInit();
  bool ServiceGimbalPitchAngleInDegCallback(
      indooruav_dji_driver::PSDK_GimbalPitchAngleInDeg::Request& request,
      indooruav_dji_driver::PSDK_GimbalPitchAngleInDeg::Response& response);

  bool initialized_;
  T_DjiOsalHandler* osal_handler_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer gimbal_pitch_angle_service_;
};

#endif  // INDOORUAV_DJI_DRIVER_MAVIC_3T_GIMBAL_CONTROLLER_H_
