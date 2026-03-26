#ifndef INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_
#define INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>

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
  struct RidInfoConfig {
    double latitude_rad;
    double longitude_rad;
    int altitude_m;
  };

  struct AxisControllerConfig {
    double kp;
    double ki;
    double kd;
    double deadband;
    double output_limit;
  };

  struct AxisControllerState {
    double integral;
    double previous_error;
    bool has_previous_error;
  };

  struct PositionControlConfig {
    int subscriber_queue_size;
    int sync_queue_size;
    double sync_max_interval_seconds;
    double control_frequency_hz;
    double odometry_timeout_seconds;
    bool send_zero_on_timeout;
    AxisControllerConfig x;
    AxisControllerConfig y;
    AxisControllerConfig z;
    AxisControllerConfig yaw;
  };

  struct FlightControllerConfig {
    std::string desired_odometry_topic;
    std::string current_odometry_topic;
    std::string takeoff_service_name;
    std::string landing_service_name;
    double rc_value_detection_frequency_hz;
    double rc_zero_deadband;
    double rc_control_return_delay_seconds;
    int motor_started_timeout_cycles;
    int takeoff_in_air_timeout_cycles;
    RidInfoConfig rid_info;
    PositionControlConfig position_control;
  };

  using OdometrySyncPolicy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>;
  using OdometrySynchronizer = message_filters::Synchronizer<OdometrySyncPolicy>;

  FlightControllerConfig LoadConfig() const;
  T_DjiReturnCode Init();
  T_DjiReturnCode DeInit();
  bool SetVelocityJoystickMode();

  void TimerRcValueDetectionCallback(const ros::TimerEvent& event);
  void TimerPositionControlCallback(const ros::TimerEvent& event);
  void CallbackSynchronizedOdometry(
      const nav_msgs::Odometry::ConstPtr& desired_message,
      const nav_msgs::Odometry::ConstPtr& current_message);
  double ComputeAxisVelocityCommand(
      const AxisControllerConfig& config,
      AxisControllerState* state,
      double raw_error,
      double dt);
  void ResetPositionControllerState();
  bool ExecuteJoystickCommandFromBodyFLU(
      double x_body_mps, double y_body_mps, double z_body_mps,
      double yaw_rate_rad_per_sec);
  bool ServiceTakeOffCallback(
      indooruav_dji_driver::PSDK_TakeOff::Request& request,
      indooruav_dji_driver::PSDK_TakeOff::Response& response);
  bool ServiceLandingCallback(
      indooruav_dji_driver::PSDK_Landing::Request& request,
      indooruav_dji_driver::PSDK_Landing::Response& response);

  bool initialized_;
  T_DjiOsalHandler* osal_handler_;
  ros::NodeHandle node_handle_;
  FlightControllerConfig config_;
  ros::Timer rc_value_detection_timer_;
  ros::Timer position_control_timer_;
  message_filters::Subscriber<nav_msgs::Odometry> desired_odometry_subscriber_;
  message_filters::Subscriber<nav_msgs::Odometry> current_odometry_subscriber_;
  std::unique_ptr<OdometrySynchronizer> odometry_synchronizer_;
  std::mutex odometry_mutex_;
  nav_msgs::Odometry latest_desired_odometry_;
  nav_msgs::Odometry latest_current_odometry_;
  ros::Time latest_synced_odometry_time_;
  ros::Time last_position_control_time_;
  AxisControllerState x_controller_state_;
  AxisControllerState y_controller_state_;
  AxisControllerState z_controller_state_;
  AxisControllerState yaw_controller_state_;
  bool has_synced_odometry_;
  bool stale_command_sent_;
  ros::ServiceServer takeoff_server_;
  ros::ServiceServer landing_server_;
};

#endif  // INDOORUAV_DJI_DRIVER_MAVIC_3T_FLIGHT_CONTROLLER_H_
