#ifndef INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_
#define INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <dji_flight_controller.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include "indooruav_dji_driver/PSDK_Landing.h"
#include "indooruav_dji_driver/PSDK_TakeOff.h"

class DjiFlightController {
 public:
  explicit DjiFlightController(T_DjiOsalHandler* osal_handler);
  DjiFlightController() = delete;
  DjiFlightController(const DjiFlightController& other) = delete;
  DjiFlightController& operator=(const DjiFlightController& other) = delete;
  ~DjiFlightController();

  bool StartTakeOff();
  bool StartForceLanding();
  bool ObtainJoystickCtrlAuthority();
  bool ReleaseJoystickCtrlAuthority();
  T_DjiOsalHandler* GetOsalHandler() const;

 private:
  T_DjiReturnCode Init();
  T_DjiReturnCode DeInit();
  bool StartLanding();
  bool StartConfirmLanding();
  bool SetVelocityJoystickMode();

  void TimerRcValueDetectionCallback(
      const ros::TimerEvent& event);
  void CallbackCommandInDjiBodyFRU(
      const geometry_msgs::TwistStamped::ConstPtr& message);
  bool ServiceTakeOffCallback(
      indooruav_dji_driver::PSDK_TakeOff::Request& request,
      indooruav_dji_driver::PSDK_TakeOff::Response& response);
  bool ServiceLandingCallback(
      indooruav_dji_driver::PSDK_Landing::Request& request,
      indooruav_dji_driver::PSDK_Landing::Response& response);

  bool initialized_;
  T_DjiOsalHandler* osal_handler_;
  ros::NodeHandle node_handle_;
  ros::Timer rc_value_detection_timer_;
  ros::Subscriber command_in_dji_body_fru_subscriber_;
  ros::ServiceServer takeoff_server_;
  ros::ServiceServer landing_server_;
};

#endif  // INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_
