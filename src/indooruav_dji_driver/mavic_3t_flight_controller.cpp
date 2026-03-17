#include "indooruav_dji_driver/mavic_3t_flight_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <dji_error.h>
#include <dji_fc_subscription.h>
#include <dji_logger.h>

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr char kDefaultCommandTopic[] = "/PSDK/DjiFlightController/CommandInDjiBodyFRU";
constexpr char kDefaultTakeoffService[] = "/PSDK/DjiFlightController/TakeOffService";
constexpr char kDefaultLandingService[] = "/PSDK/DjiFlightController/LandingService";

constexpr double kDefaultRcValueDetectionFrequencyHz = 10.0;
constexpr double kDefaultRcZeroDeadband = 0.02;
constexpr double kRcControlReturnDelaySeconds = 5.0;
constexpr int kMotorStartedTimeoutCycles = 20;
constexpr int kTakeoffInAirTimeoutCycles = 110;

std::string GetStringParam(const std::string& suffix, const std::string& default_value) {
  std::string value = default_value;
  ros::param::param<std::string>(std::string(kParamPrefix) + suffix, value, default_value);
  return value;
}

double GetDoubleParam(const std::string& suffix, double default_value) {
  double value = default_value;
  ros::param::param<double>(std::string(kParamPrefix) + suffix, value, default_value);
  return value;
}

bool IsSuccessOrUnsupported(T_DjiReturnCode return_code) {
  return return_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || return_code == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
}

void LogOptionalFeatureResult(const char* feature_name, T_DjiReturnCode return_code) {
  if (return_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return;
  }

  if (return_code == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
    USER_LOG_WARN("%s is not supported on this aircraft.", feature_name);
    return;
  }

  USER_LOG_WARN("%s failed, error code: 0x%08X", feature_name, return_code);
}

bool LoadRidInfo(T_DjiFlightControllerRidInfo& rid_info) {
  rid_info.latitude = GetDoubleParam("/parameters/rid_info/latitude_rad", 0.0);
  rid_info.longitude = GetDoubleParam("/parameters/rid_info/longitude_rad", 0.0);

  int altitude = 0;
  ros::param::param<int>(std::string(kParamPrefix) + "/parameters/rid_info/altitude_m", altitude, 0);
  if (altitude < 0) altitude = 0;
  rid_info.altitude = static_cast<uint16_t>(altitude);
  return true;
}

bool IsRcStickNeutral(const T_DjiFcSubscriptionRCWithFlagData& rc_with_flag_data, double deadband) {
  return std::fabs(rc_with_flag_data.pitch) <= deadband &&
         std::fabs(rc_with_flag_data.roll) <= deadband &&
         std::fabs(rc_with_flag_data.yaw) <= deadband &&
         std::fabs(rc_with_flag_data.throttle) <= deadband;
}

bool IsTakeoffDisplayMode(T_DjiFcSubscriptionDisplaymode display_mode) {
  return display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
         display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF;
}

T_DjiReturnCode Dji_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo event_data) {
  USER_LOG_INFO(
      "Joystick authority switched, authority=%d, reason=%d.",
      event_data.curJoystickCtrlAuthority,
      event_data.joystickCtrlAuthoritySwitchEvent);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiFcSubscriptionFlightStatus GetLatestValueOfFlightState() {
  T_DjiFcSubscriptionFlightStatus flight_status = DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED;
  T_DjiDataTimestamp flight_status_timestamp = {};

  const T_DjiReturnCode return_code = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
      reinterpret_cast<uint8_t*>(&flight_status), sizeof(flight_status),
      &flight_status_timestamp);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get flight status failed, error code: 0x%08X", return_code);
  }

  return flight_status;
}

T_DjiFcSubscriptionDisplaymode GetLatestValueOfDisplayMode() {
  T_DjiFcSubscriptionDisplaymode display_mode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL;
  T_DjiDataTimestamp display_mode_timestamp = {};

  const T_DjiReturnCode return_code = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
      reinterpret_cast<uint8_t*>(&display_mode), sizeof(display_mode),
      &display_mode_timestamp);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get display mode failed, error code: 0x%08X", return_code);
  }

  return display_mode;
}

T_DjiFcSubscriptionRCWithFlagData GetLatestValueOfRCWithFlagData() {
  T_DjiFcSubscriptionRCWithFlagData rc_with_flag_data = {};
  T_DjiDataTimestamp rc_with_flag_data_timestamp = {};

  const T_DjiReturnCode return_code = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
      reinterpret_cast<uint8_t*>(&rc_with_flag_data),
      sizeof(rc_with_flag_data), &rc_with_flag_data_timestamp);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get RC-with-flag data failed, error code: 0x%08X", return_code);
  }

  return rc_with_flag_data;
}

T_DjiFcSubscriptionControlDevice GetLatestValueOfControlDevice() {
  T_DjiFcSubscriptionControlDevice control_device = {};
  T_DjiDataTimestamp control_device_timestamp = {};

  const T_DjiReturnCode return_code = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
      reinterpret_cast<uint8_t*>(&control_device), sizeof(control_device),
      &control_device_timestamp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get control device failed, error code: 0x%08X", return_code);
  }

  return control_device;
}

bool MotorStartedCheck(const DjiFlightController& controller) {
  int cycles = 0;
  while ((GetLatestValueOfFlightState() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND ||
          GetLatestValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START) &&
         cycles < kMotorStartedTimeoutCycles) 
  {
    ++cycles;
    controller.GetOsalHandler()->TaskSleepMs(100);
  }
  return cycles < kMotorStartedTimeoutCycles;
}

bool TakeOffInAirCheck(const DjiFlightController& controller) {
  int cycles = 0;
  while (GetLatestValueOfFlightState() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
         !IsTakeoffDisplayMode(GetLatestValueOfDisplayMode()) &&
         cycles < kTakeoffInAirTimeoutCycles) 
  {
    ++cycles;
    controller.GetOsalHandler()->TaskSleepMs(100);
  }
  return cycles < kTakeoffInAirTimeoutCycles;
}

bool TakeoffFinishedCheck(const DjiFlightController& controller) {
  while (IsTakeoffDisplayMode(GetLatestValueOfDisplayMode())) {
    controller.GetOsalHandler()->TaskSleepMs(1000);
  }

  const T_DjiFcSubscriptionDisplaymode display_mode = GetLatestValueOfDisplayMode();
  return display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
         display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE ||
         display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL;
}

}  // namespace











































DjiFlightController::DjiFlightController(T_DjiOsalHandler* osal_handler)
    : initialized_(false), osal_handler_(osal_handler), node_handle_() {

  initialized_ = Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

  const double detection_frequency_hz = std::max(GetDoubleParam("/parameters/rc_value_detection_frequency_hz", kDefaultRcValueDetectionFrequencyHz), 1.0);
  rc_value_detection_timer_ = node_handle_.createTimer(ros::Duration(1.0 / detection_frequency_hz), &DjiFlightController::TimerRcValueDetectionCallback, this);
  rc_value_detection_timer_.stop();

  command_in_dji_body_fru_subscriber_ = node_handle_.subscribe(
      GetStringParam("/topics/command_in_dji_body_fru", kDefaultCommandTopic),
      10, &DjiFlightController::CallbackCommandInDjiBodyFRU, this);
  takeoff_server_ = node_handle_.advertiseService(
      GetStringParam("/services/takeoff", kDefaultTakeoffService),
      &DjiFlightController::ServiceTakeOffCallback, this);
  landing_server_ = node_handle_.advertiseService(
      GetStringParam("/services/landing", kDefaultLandingService),
      &DjiFlightController::ServiceLandingCallback, this);
}

DjiFlightController::~DjiFlightController() {
  if (initialized_) {
    DeInit();
  }
}

T_DjiOsalHandler* DjiFlightController::GetOsalHandler() const {
  return osal_handler_;
}

T_DjiReturnCode DjiFlightController::Init() {
  if (osal_handler_ == nullptr) {
    USER_LOG_ERROR("OSAL handler is null.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
  }

  T_DjiFlightControllerRidInfo rid_info = {};
  LoadRidInfo(rid_info);

  T_DjiReturnCode return_code = DjiFlightController_Init(rid_info);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init flight controller failed, error code: 0x%08X", return_code);
    return return_code;
  }

  return_code = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(Dji_FlightControlJoystickCtrlAuthSwitchEventCallback);
  if (!IsSuccessOrUnsupported(return_code)) {
    USER_LOG_ERROR("Register joystick authority callback failed, error code: 0x%08X", return_code);
    return return_code;
  }

  LogOptionalFeatureResult(
      "Set RC lost action",
      DjiFlightController_SetRCLostAction(
          DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER));
  LogOptionalFeatureResult(
      "Enable RC lost action",
      DjiFlightController_SetRCLostActionEnableStatus(
          DJI_FLIGHT_CONTROLLER_ENABLE_RC_LOST_ACTION));
  LogOptionalFeatureResult(
      "Obtain joystick authority",
      DjiFlightController_ObtainJoystickCtrlAuthority());
  LogOptionalFeatureResult(
      "Disable RTK position",
      DjiFlightController_SetRtkPositionEnableStatus(
          DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION));
  LogOptionalFeatureResult(
      "Disable horizontal obstacle avoidance",
      DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
          DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE));
  LogOptionalFeatureResult(
      "Disable upwards obstacle avoidance",
      DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
          DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE));
  LogOptionalFeatureResult(
      "Disable downwards obstacle avoidance",
      DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
          DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE));

  SetVelocityJoystickMode();
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiFlightController::DeInit() {
  LogOptionalFeatureResult(
      "Release joystick authority",
      DjiFlightController_ReleaseJoystickCtrlAuthority());

  const T_DjiReturnCode return_code = DjiFlightController_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Flight controller deinit failed, error code: 0x%08X", return_code);
  }

  return return_code;
}

bool DjiFlightController::StartTakeOff() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_StartTakeoff();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Request takeoff failed, error code: 0x%08X", return_code);
    return false;
  }

  if (!MotorStartedCheck(*this)) {
    USER_LOG_ERROR("Takeoff failed because the motors never started.");
    return false;
  }

  if (!TakeOffInAirCheck(*this)) {
    USER_LOG_ERROR("Takeoff failed because the aircraft stayed on ground.");
    return false;
  }

  if (!TakeoffFinishedCheck(*this)) {
    USER_LOG_ERROR("Takeoff ended in an unexpected display mode.");
    return false;
  }

  rc_value_detection_timer_.start();
  USER_LOG_INFO("Takeoff successful.");
  return true;
}

bool DjiFlightController::StartLanding() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_StartLanding();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Request landing failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiFlightController::StartConfirmLanding() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_StartConfirmLanding();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Confirm landing failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiFlightController::StartForceLanding() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_StartForceLanding();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Request force landing failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiFlightController::ObtainJoystickCtrlAuthority() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", return_code);
    return false;
  }

  SetVelocityJoystickMode();
  return true;
}

bool DjiFlightController::ReleaseJoystickCtrlAuthority() {
  if (!initialized_) {
    USER_LOG_ERROR("Flight controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiFlightController_ReleaseJoystickCtrlAuthority();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiFlightController::SetVelocityJoystickMode() {
  T_DjiFlightControllerJoystickMode joystick_mode = {};
  joystick_mode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE;
  joystick_mode.verticalControlMode   = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE;
  joystick_mode.yawControlMode        = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;
  joystick_mode.horizontalCoordinate  = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;
  joystick_mode.stableControlMode     = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE;
  DjiFlightController_SetJoystickMode(joystick_mode);
  return true;
}

void DjiFlightController::TimerRcValueDetectionCallback(const ros::TimerEvent& event) {
  (void)event;
  if (!initialized_) {
    return;
  }

  static ros::Time last_zero_time;
  const double rc_zero_deadband = GetDoubleParam("/parameters/rc_zero_deadband", kDefaultRcZeroDeadband);

  const T_DjiFcSubscriptionRCWithFlagData rc_with_flag_data = GetLatestValueOfRCWithFlagData();
  const T_DjiFcSubscriptionControlDevice control_device = GetLatestValueOfControlDevice();

  if (last_zero_time.isZero()) {
    last_zero_time = ros::Time::now();
  }

  if (rc_with_flag_data.flag.logicConnected == 0) {
    ROS_WARN_THROTTLE(1.0, "RC and flight controller are disconnected.");
    return;
  }

  if (!IsRcStickNeutral(rc_with_flag_data, rc_zero_deadband) && control_device.controlAuthority == DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_PSDK) {
    USER_LOG_INFO("RC input detected, releasing joystick authority back to the RC.");
    ReleaseJoystickCtrlAuthority();
    last_zero_time = ros::Time::now();
    return;
  }

  if (control_device.controlAuthority == DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_PSDK) {
    last_zero_time = ros::Time::now();
    return;
  }

  if (control_device.controlAuthority != DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_RC) {
    last_zero_time = ros::Time::now();
    return;
  }

  if (!IsRcStickNeutral(rc_with_flag_data, rc_zero_deadband)) {
    last_zero_time = ros::Time::now();
    return;
  }

  const ros::Time current_time = ros::Time::now();
  if ((current_time - last_zero_time).toSec() >= kRcControlReturnDelaySeconds) {
    USER_LOG_INFO("RC sticks have been neutral for 5 seconds, reclaiming joystick authority.");
    ObtainJoystickCtrlAuthority();
    last_zero_time = current_time;
  }
}

void DjiFlightController::CallbackCommandInDjiBodyFRU(const geometry_msgs::TwistStamped::ConstPtr& message) {
  if (!initialized_) {
    return;
  }

  T_DjiFlightControllerJoystickCommand joystick_command = {};
  joystick_command.x = static_cast<dji_f32_t>(message->twist.linear.x);
  joystick_command.y = static_cast<dji_f32_t>(message->twist.linear.y);
  joystick_command.z = static_cast<dji_f32_t>(message->twist.linear.z);
  joystick_command.yaw = static_cast<dji_f32_t>(message->twist.angular.z);

  const T_DjiReturnCode return_code = DjiFlightController_ExecuteJoystickAction(joystick_command);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_WARN("Execute joystick action failed, error code: 0x%08X", return_code);
  }
}

bool DjiFlightController::ServiceTakeOffCallback(
    indooruav_dji_driver::PSDK_TakeOff::Request& request,
    indooruav_dji_driver::PSDK_TakeOff::Response& response) 
{
  USER_LOG_INFO("------ServiceTakeOffCallback start------");
  if (!request.takeoff) {
    USER_LOG_ERROR("Received false takeoff request, nothing to do.");
    response.result = false;
    return true;
  }

  response.result = StartTakeOff();
  USER_LOG_INFO("------ServiceTakeOffCallback end------");
  return true;
}

bool DjiFlightController::ServiceLandingCallback(
    indooruav_dji_driver::PSDK_Landing::Request& request,
    indooruav_dji_driver::PSDK_Landing::Response& response) 
{
  USER_LOG_INFO("------ServiceLandingCallback start------");
  if (!request.landing) {
    USER_LOG_ERROR("Received false landing request, nothing to do.");
    response.result = false;
    return true;
  }

  response.result = StartForceLanding();
  USER_LOG_INFO("------ServiceLandingCallback end------");
  return true;
}
