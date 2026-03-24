#include "indooruav_dji_driver/mavic_3t_flight_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <boost/bind/bind.hpp>

#include <dji_error.h>
#include <dji_fc_subscription.h>
#include <dji_logger.h>

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr double kDegreesPerRadian = 57.29577951308232;
constexpr double kPi = 3.14159265358979323846;

template <typename T>
T GetRequiredParam(const std::string& suffix) {
  const std::string full_name = std::string(kParamPrefix) + suffix;

  T value{};
  if (!ros::param::get(full_name, value)) {
    throw std::runtime_error("Missing required ROS parameter: " + full_name);
  }

  return value;
}

void ValidateNonEmptyString(
    const std::string& param_name, const std::string& value) {
  if (value.empty()) {
    throw std::runtime_error(
        "ROS parameter must not be empty: " + param_name);
  }
}

void ValidatePositiveDouble(const std::string& param_name, double value) {
  if (value <= 0.0) {
    throw std::runtime_error(
        "ROS parameter must be > 0: " + param_name);
  }
}

void ValidatePositiveInt(const std::string& param_name, int value) {
  if (value <= 0) {
    throw std::runtime_error(
        "ROS parameter must be > 0: " + param_name);
  }
}

void ValidateNonNegativeDouble(const std::string& param_name, double value) {
  if (value < 0.0) {
    throw std::runtime_error(
        "ROS parameter must be >= 0: " + param_name);
  }
}

void ValidateNonNegativeInt(const std::string& param_name, int value) {
  if (value < 0) {
    throw std::runtime_error(
        "ROS parameter must be >= 0: " + param_name);
  }
}

double ClampMagnitude(double value, double limit) {
  if (limit <= 0.0) {
    return 0.0;
  }

  return std::max(-limit, std::min(value, limit));
}

double ApplyDeadband(double value, double deadband) {
  return std::fabs(value) <= deadband ? 0.0 : value;
}

double NormalizeAngleRadians(double angle) {
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }

  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }

  return angle;
}

double GetYawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double siny_cosp =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void TransformWorldFluVelocityToBodyFlu(
    double x_world_mps, double y_world_mps, double current_yaw_rad,
    double* x_body_mps, double* y_body_mps) {
  const double cos_yaw = std::cos(current_yaw_rad);
  const double sin_yaw = std::sin(current_yaw_rad);

  *x_body_mps = cos_yaw * x_world_mps + sin_yaw * y_world_mps;
  *y_body_mps = -sin_yaw * x_world_mps + cos_yaw * y_world_mps;
}

bool IsSuccessOrUnsupported(T_DjiReturnCode return_code) {
  return return_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
         return_code == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
}

void LogOptionalFeatureResult(
    const char* feature_name, T_DjiReturnCode return_code) {
  if (return_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return;
  }

  if (return_code == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
    USER_LOG_WARN("%s is not supported on this aircraft.", feature_name);
    return;
  }

  USER_LOG_WARN("%s failed, error code: 0x%08X", feature_name, return_code);
}

bool IsRcStickNeutral(
    const T_DjiFcSubscriptionRCWithFlagData& rc_with_flag_data,
    double deadband) {
  return std::fabs(rc_with_flag_data.pitch) <= deadband &&
         std::fabs(rc_with_flag_data.roll) <= deadband &&
         std::fabs(rc_with_flag_data.yaw) <= deadband &&
         std::fabs(rc_with_flag_data.throttle) <= deadband;
}

bool IsTakeoffDisplayMode(T_DjiFcSubscriptionDisplaymode display_mode) {
  return display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
         display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF;
}

T_DjiReturnCode Dji_FlightControlJoystickCtrlAuthSwitchEventCallback(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo event_data) {
  USER_LOG_INFO(
      "Joystick authority switched, authority=%d, reason=%d.",
      event_data.curJoystickCtrlAuthority,
      event_data.joystickCtrlAuthoritySwitchEvent);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiFcSubscriptionFlightStatus GetLatestValueOfFlightState() {
  T_DjiFcSubscriptionFlightStatus flight_status =
      DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED;
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
  T_DjiFcSubscriptionDisplaymode display_mode =
      DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL;
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
    USER_LOG_ERROR(
        "Get RC-with-flag data failed, error code: 0x%08X", return_code);
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

bool MotorStartedCheck(
    const DjiFlightController& controller, int timeout_cycles) {
  int cycles = 0;
  while ((GetLatestValueOfFlightState() !=
              DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND ||
          GetLatestValueOfDisplayMode() !=
              DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START) &&
         cycles < timeout_cycles) {
    ++cycles;
    controller.GetOsalHandler()->TaskSleepMs(100);
  }
  return cycles < timeout_cycles;
}

bool TakeOffInAirCheck(
    const DjiFlightController& controller, int timeout_cycles) {
  int cycles = 0;
  while (GetLatestValueOfFlightState() !=
             DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
         !IsTakeoffDisplayMode(GetLatestValueOfDisplayMode()) &&
         cycles < timeout_cycles) {
    ++cycles;
    controller.GetOsalHandler()->TaskSleepMs(100);
  }
  return cycles < timeout_cycles;
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
    : initialized_(false),
      osal_handler_(osal_handler),
      node_handle_(),
      config_(),
      desired_odometry_subscriber_(),
      current_odometry_subscriber_(),
      odometry_synchronizer_(),
      latest_synced_odometry_time_(),
      has_synced_odometry_(false),
      stale_command_sent_(false) {
  config_ = LoadConfig();
  initialized_ = Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

  rc_value_detection_timer_ = node_handle_.createTimer(
      ros::Duration(1.0 / config_.rc_value_detection_frequency_hz),
      &DjiFlightController::TimerRcValueDetectionCallback, this);
  rc_value_detection_timer_.stop();

  if (config_.position_control.enabled) {
    position_control_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / config_.position_control.control_frequency_hz),
        &DjiFlightController::TimerPositionControlCallback, this);

    desired_odometry_subscriber_.subscribe(
        node_handle_, config_.desired_odometry_topic,
        static_cast<uint32_t>(config_.position_control.subscriber_queue_size));
    current_odometry_subscriber_.subscribe(
        node_handle_, config_.current_odometry_topic,
        static_cast<uint32_t>(config_.position_control.subscriber_queue_size));

    odometry_synchronizer_.reset(new OdometrySynchronizer(
        OdometrySyncPolicy(config_.position_control.sync_queue_size),
        desired_odometry_subscriber_, current_odometry_subscriber_));
    odometry_synchronizer_->setMaxIntervalDuration(
        ros::Duration(config_.position_control.sync_max_interval_seconds));
    odometry_synchronizer_->registerCallback(boost::bind(
        &DjiFlightController::CallbackSynchronizedOdometry, this,
        boost::placeholders::_1, boost::placeholders::_2));

    USER_LOG_INFO(
        "Position control enabled. desired odometry topic: %s, current odometry topic: %s.",
        config_.desired_odometry_topic.c_str(),
        config_.current_odometry_topic.c_str());
  } else {
    command_in_body_flu_subscriber_ = node_handle_.subscribe(
        config_.command_in_body_flu_topic,
        static_cast<uint32_t>(config_.command_subscriber_queue_size),
        &DjiFlightController::CallbackCommandInBodyFLU, this);

    USER_LOG_INFO(
        "Direct FLU velocity command mode enabled. command topic: %s.",
        config_.command_in_body_flu_topic.c_str());
  }

  takeoff_server_ = node_handle_.advertiseService(
      config_.takeoff_service_name,
      &DjiFlightController::ServiceTakeOffCallback, this);
  landing_server_ = node_handle_.advertiseService(
      config_.landing_service_name,
      &DjiFlightController::ServiceLandingCallback, this);
}

DjiFlightController::~DjiFlightController() {
  if (initialized_) {
    DeInit();
  }
}

DjiFlightController::FlightControllerConfig DjiFlightController::LoadConfig() const {
  FlightControllerConfig config = {};

  config.command_in_body_flu_topic =
      GetRequiredParam<std::string>("/topics/command_in_body_flu");
  config.desired_odometry_topic =
      GetRequiredParam<std::string>("/topics/desired_odometry");
  config.current_odometry_topic =
      GetRequiredParam<std::string>("/topics/current_odometry");
  config.takeoff_service_name =
      GetRequiredParam<std::string>("/services/takeoff");
  config.landing_service_name =
      GetRequiredParam<std::string>("/services/landing");

  ValidateNonEmptyString(
      "/topics/command_in_body_flu", config.command_in_body_flu_topic);
  ValidateNonEmptyString(
      "/topics/desired_odometry", config.desired_odometry_topic);
  ValidateNonEmptyString(
      "/topics/current_odometry", config.current_odometry_topic);
  ValidateNonEmptyString(
      "/services/takeoff", config.takeoff_service_name);
  ValidateNonEmptyString(
      "/services/landing", config.landing_service_name);

  config.rc_value_detection_frequency_hz = GetRequiredParam<double>(
      "/parameters/rc_value_detection_frequency_hz");
  config.rc_zero_deadband =
      GetRequiredParam<double>("/parameters/rc_zero_deadband");
  config.rc_control_return_delay_seconds = GetRequiredParam<double>(
      "/parameters/rc_control_return_delay_s");
  config.command_subscriber_queue_size = GetRequiredParam<int>(
      "/parameters/command_subscriber_queue_size");
  config.motor_started_timeout_cycles = GetRequiredParam<int>(
      "/parameters/takeoff/motor_started_timeout_cycles");
  config.takeoff_in_air_timeout_cycles = GetRequiredParam<int>(
      "/parameters/takeoff/in_air_timeout_cycles");

  ValidatePositiveDouble(
      "/parameters/rc_value_detection_frequency_hz",
      config.rc_value_detection_frequency_hz);
  ValidateNonNegativeDouble(
      "/parameters/rc_zero_deadband", config.rc_zero_deadband);
  ValidatePositiveDouble(
      "/parameters/rc_control_return_delay_s",
      config.rc_control_return_delay_seconds);
  ValidatePositiveInt(
      "/parameters/command_subscriber_queue_size",
      config.command_subscriber_queue_size);
  ValidatePositiveInt(
      "/parameters/takeoff/motor_started_timeout_cycles",
      config.motor_started_timeout_cycles);
  ValidatePositiveInt(
      "/parameters/takeoff/in_air_timeout_cycles",
      config.takeoff_in_air_timeout_cycles);

  config.rid_info.latitude_rad =
      GetRequiredParam<double>("/parameters/rid_info/latitude_rad");
  config.rid_info.longitude_rad =
      GetRequiredParam<double>("/parameters/rid_info/longitude_rad");
  config.rid_info.altitude_m =
      GetRequiredParam<int>("/parameters/rid_info/altitude_m");
  ValidateNonNegativeInt(
      "/parameters/rid_info/altitude_m", config.rid_info.altitude_m);
  if (config.rid_info.altitude_m >
      static_cast<int>(std::numeric_limits<uint16_t>::max())) {
    throw std::runtime_error(
        "ROS parameter exceeds uint16 range: "
        "/parameters/rid_info/altitude_m");
  }

  config.position_control.enabled =
      GetRequiredParam<bool>("/parameters/position_control/enabled");
  config.position_control.subscriber_queue_size = GetRequiredParam<int>(
      "/parameters/position_control/subscriber_queue_size");
  config.position_control.sync_queue_size = GetRequiredParam<int>(
      "/parameters/position_control/sync_queue_size");
  config.position_control.sync_max_interval_seconds = GetRequiredParam<double>(
      "/parameters/position_control/sync_max_interval_s");
  config.position_control.control_frequency_hz = GetRequiredParam<double>(
      "/parameters/position_control/control_frequency_hz");
  config.position_control.odometry_timeout_seconds = GetRequiredParam<double>(
      "/parameters/position_control/odometry_timeout_s");
  config.position_control.send_zero_on_timeout = GetRequiredParam<bool>(
      "/parameters/position_control/send_zero_on_timeout");

  ValidatePositiveInt(
      "/parameters/position_control/subscriber_queue_size",
      config.position_control.subscriber_queue_size);
  ValidatePositiveInt(
      "/parameters/position_control/sync_queue_size",
      config.position_control.sync_queue_size);
  ValidatePositiveDouble(
      "/parameters/position_control/sync_max_interval_s",
      config.position_control.sync_max_interval_seconds);
  ValidatePositiveDouble(
      "/parameters/position_control/control_frequency_hz",
      config.position_control.control_frequency_hz);
  ValidatePositiveDouble(
      "/parameters/position_control/odometry_timeout_s",
      config.position_control.odometry_timeout_seconds);

  config.position_control.x.gain =
      GetRequiredParam<double>("/parameters/position_control/x/gain");
  config.position_control.x.deadband =
      GetRequiredParam<double>("/parameters/position_control/x/deadband_m");
  config.position_control.x.output_limit = GetRequiredParam<double>(
      "/parameters/position_control/x/output_limit_mps");
  config.position_control.y.gain =
      GetRequiredParam<double>("/parameters/position_control/y/gain");
  config.position_control.y.deadband =
      GetRequiredParam<double>("/parameters/position_control/y/deadband_m");
  config.position_control.y.output_limit = GetRequiredParam<double>(
      "/parameters/position_control/y/output_limit_mps");
  config.position_control.z.gain =
      GetRequiredParam<double>("/parameters/position_control/z/gain");
  config.position_control.z.deadband =
      GetRequiredParam<double>("/parameters/position_control/z/deadband_m");
  config.position_control.z.output_limit = GetRequiredParam<double>(
      "/parameters/position_control/z/output_limit_mps");
  config.position_control.yaw.gain =
      GetRequiredParam<double>("/parameters/position_control/yaw/gain");
  config.position_control.yaw.deadband = GetRequiredParam<double>(
      "/parameters/position_control/yaw/deadband_rad");
  config.position_control.yaw.output_limit = GetRequiredParam<double>(
      "/parameters/position_control/yaw/output_limit_radps");

  ValidateNonNegativeDouble(
      "/parameters/position_control/x/gain",
      config.position_control.x.gain);
  ValidateNonNegativeDouble(
      "/parameters/position_control/x/deadband_m",
      config.position_control.x.deadband);
  ValidateNonNegativeDouble(
      "/parameters/position_control/x/output_limit_mps",
      config.position_control.x.output_limit);
  ValidateNonNegativeDouble(
      "/parameters/position_control/y/gain",
      config.position_control.y.gain);
  ValidateNonNegativeDouble(
      "/parameters/position_control/y/deadband_m",
      config.position_control.y.deadband);
  ValidateNonNegativeDouble(
      "/parameters/position_control/y/output_limit_mps",
      config.position_control.y.output_limit);
  ValidateNonNegativeDouble(
      "/parameters/position_control/z/gain",
      config.position_control.z.gain);
  ValidateNonNegativeDouble(
      "/parameters/position_control/z/deadband_m",
      config.position_control.z.deadband);
  ValidateNonNegativeDouble(
      "/parameters/position_control/z/output_limit_mps",
      config.position_control.z.output_limit);
  ValidateNonNegativeDouble(
      "/parameters/position_control/yaw/gain",
      config.position_control.yaw.gain);
  ValidateNonNegativeDouble(
      "/parameters/position_control/yaw/deadband_rad",
      config.position_control.yaw.deadband);
  ValidateNonNegativeDouble(
      "/parameters/position_control/yaw/output_limit_radps",
      config.position_control.yaw.output_limit);

  return config;
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
  rid_info.latitude = config_.rid_info.latitude_rad;
  rid_info.longitude = config_.rid_info.longitude_rad;
  rid_info.altitude = static_cast<uint16_t>(config_.rid_info.altitude_m);

  T_DjiReturnCode return_code = DjiFlightController_Init(rid_info);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init flight controller failed, error code: 0x%08X", return_code);
    return return_code;
  }

  return_code = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
      Dji_FlightControlJoystickCtrlAuthSwitchEventCallback);
  if (!IsSuccessOrUnsupported(return_code)) {
    USER_LOG_ERROR(
        "Register joystick authority callback failed, error code: 0x%08X",
        return_code);
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

  if (!MotorStartedCheck(*this, config_.motor_started_timeout_cycles)) {
    USER_LOG_ERROR("Takeoff failed because the motors never started.");
    return false;
  }

  if (!TakeOffInAirCheck(*this, config_.takeoff_in_air_timeout_cycles)) {
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
  joystick_mode.horizontalControlMode =
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE;
  joystick_mode.verticalControlMode =
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE;
  joystick_mode.yawControlMode =
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;
  joystick_mode.horizontalCoordinate =
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;
  joystick_mode.stableControlMode =
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE;
  DjiFlightController_SetJoystickMode(joystick_mode);
  return true;
}

void DjiFlightController::TimerRcValueDetectionCallback(
    const ros::TimerEvent& event) {
  (void)event;
  if (!initialized_) {
    return;
  }

  static ros::Time last_zero_time;

  const T_DjiFcSubscriptionRCWithFlagData rc_with_flag_data =
      GetLatestValueOfRCWithFlagData();
  const T_DjiFcSubscriptionControlDevice control_device =
      GetLatestValueOfControlDevice();

  if (last_zero_time.isZero()) {
    last_zero_time = ros::Time::now();
  }

  if (rc_with_flag_data.flag.logicConnected == 0) {
    ROS_WARN_THROTTLE(1.0, "RC and flight controller are disconnected.");
    return;
  }

  if (!IsRcStickNeutral(rc_with_flag_data, config_.rc_zero_deadband) &&
      control_device.controlAuthority ==
          DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_PSDK) {
    USER_LOG_INFO("RC input detected, releasing joystick authority back to the RC.");
    ReleaseJoystickCtrlAuthority();
    last_zero_time = ros::Time::now();
    return;
  }

  if (control_device.controlAuthority ==
      DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_PSDK) {
    last_zero_time = ros::Time::now();
    return;
  }

  if (control_device.controlAuthority !=
      DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_RC) {
    last_zero_time = ros::Time::now();
    return;
  }

  if (!IsRcStickNeutral(rc_with_flag_data, config_.rc_zero_deadband)) {
    last_zero_time = ros::Time::now();
    return;
  }

  const ros::Time current_time = ros::Time::now();
  if ((current_time - last_zero_time).toSec() >=
      config_.rc_control_return_delay_seconds) {
    USER_LOG_INFO(
        "RC sticks have been neutral for %.2f seconds, reclaiming joystick authority.",
        config_.rc_control_return_delay_seconds);
    ObtainJoystickCtrlAuthority();
    last_zero_time = current_time;
  }
}

void DjiFlightController::TimerPositionControlCallback(
    const ros::TimerEvent& event) {
  (void)event;
  if (!initialized_ || !config_.position_control.enabled) {
    return;
  }

  nav_msgs::Odometry desired_odometry;
  nav_msgs::Odometry current_odometry;
  ros::Time latest_sync_time;

  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    if (!has_synced_odometry_) {
      ROS_WARN_THROTTLE(
          1.0,
          "Waiting for synchronized desired/current odometry before running position control.");
      return;
    }

    desired_odometry = latest_desired_odometry_;
    current_odometry = latest_current_odometry_;
    latest_sync_time = latest_synced_odometry_time_;
  }

  const double odometry_age_seconds =
      (ros::Time::now() - latest_sync_time).toSec();
  if (odometry_age_seconds > config_.position_control.odometry_timeout_seconds) {
    ROS_WARN_THROTTLE(
        1.0,
        "Synchronized odometry is stale (age=%.3f s, timeout=%.3f s).",
        odometry_age_seconds,
        config_.position_control.odometry_timeout_seconds);

    bool should_send_zero = false;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      should_send_zero =
          config_.position_control.send_zero_on_timeout && !stale_command_sent_;
      if (should_send_zero) {
        stale_command_sent_ = true;
      }
    }

    if (should_send_zero) {
      ExecuteJoystickCommandInBodyFLU(0.0, 0.0, 0.0, 0.0);
    }

    return;
  }

  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    stale_command_sent_ = false;
  }

  const double x_error_world = ApplyDeadband(
      desired_odometry.pose.pose.position.x - current_odometry.pose.pose.position.x,
      config_.position_control.x.deadband);
  const double y_error_world = ApplyDeadband(
      desired_odometry.pose.pose.position.y - current_odometry.pose.pose.position.y,
      config_.position_control.y.deadband);
  const double z_error_world = ApplyDeadband(
      desired_odometry.pose.pose.position.z - current_odometry.pose.pose.position.z,
      config_.position_control.z.deadband);

  const double desired_yaw =
      GetYawFromQuaternion(desired_odometry.pose.pose.orientation);
  const double current_yaw =
      GetYawFromQuaternion(current_odometry.pose.pose.orientation);
  const double yaw_error = ApplyDeadband(
      NormalizeAngleRadians(desired_yaw - current_yaw),
      config_.position_control.yaw.deadband);

  const double x_velocity_world = ClampMagnitude(
      config_.position_control.x.gain * x_error_world,
      config_.position_control.x.output_limit);
  const double y_velocity_world = ClampMagnitude(
      config_.position_control.y.gain * y_error_world,
      config_.position_control.y.output_limit);
  const double z_velocity_world = ClampMagnitude(
      config_.position_control.z.gain * z_error_world,
      config_.position_control.z.output_limit);
  const double yaw_rate_command = ClampMagnitude(
      config_.position_control.yaw.gain * yaw_error,
      config_.position_control.yaw.output_limit);

  double x_velocity_body = 0.0;
  double y_velocity_body = 0.0;
  TransformWorldFluVelocityToBodyFlu(
      x_velocity_world, y_velocity_world, current_yaw,
      &x_velocity_body, &y_velocity_body);

  ExecuteJoystickCommandInBodyFLU(
      x_velocity_body, y_velocity_body, z_velocity_world,
      yaw_rate_command);
}

void DjiFlightController::CallbackSynchronizedOdometry(
    const nav_msgs::Odometry::ConstPtr& desired_message,
    const nav_msgs::Odometry::ConstPtr& current_message) {
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  latest_desired_odometry_ = *desired_message;
  latest_current_odometry_ = *current_message;
  latest_synced_odometry_time_ = ros::Time::now();
  has_synced_odometry_ = true;
  stale_command_sent_ = false;
}

bool DjiFlightController::ExecuteJoystickCommandInBodyFLU(
    double x_body_mps, double y_body_mps, double z_body_mps,
    double yaw_rate_rad_per_sec) {
  if (!initialized_) {
    return false;
  }

  T_DjiFlightControllerJoystickCommand joystick_command = {};
  joystick_command.x = static_cast<dji_f32_t>(x_body_mps);
  joystick_command.y = static_cast<dji_f32_t>(-y_body_mps);
  joystick_command.z = static_cast<dji_f32_t>(z_body_mps);
  joystick_command.yaw = static_cast<dji_f32_t>(-yaw_rate_rad_per_sec * kDegreesPerRadian);

  const T_DjiReturnCode return_code =
      DjiFlightController_ExecuteJoystickAction(joystick_command);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_WARN(
        "Execute joystick action failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

void DjiFlightController::CallbackCommandInBodyFLU(
    const geometry_msgs::TwistStamped::ConstPtr& message) {
  ExecuteJoystickCommandInBodyFLU(
      message->twist.linear.x,
      message->twist.linear.y,
      message->twist.linear.z,
      message->twist.angular.z);
}

bool DjiFlightController::ServiceTakeOffCallback(
    indooruav_dji_driver::PSDK_TakeOff::Request& request,
    indooruav_dji_driver::PSDK_TakeOff::Response& response) {
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
    indooruav_dji_driver::PSDK_Landing::Response& response) {
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
