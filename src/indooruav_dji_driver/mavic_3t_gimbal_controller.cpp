#include "indooruav_dji_driver/mavic_3t_gimbal_controller.h"

#include <cmath>
#include <string>

#include <dji_fc_subscription.h>
#include <dji_logger.h>

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr char kDefaultGimbalPitchService[] =
    "/PSDK/DjiGimbalController/GimbalPitchAngleInDegService";
constexpr E_DjiMountPosition kGimbalMountPosition =
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
constexpr double kDegreesPerRadian = 57.29577951308232;

std::string GetStringParam(const std::string& suffix,
                           const std::string& default_value) {
  std::string value = default_value;
  ros::param::param<std::string>(std::string(kParamPrefix) + suffix, value,
                                 default_value);
  return value;
}

T_DjiFcSubscriptionQuaternion GetLatestValueOfQuaternion() {
  T_DjiFcSubscriptionQuaternion quaternion = {};
  T_DjiDataTimestamp quaternion_timestamp = {};

  const T_DjiReturnCode return_code = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
      reinterpret_cast<uint8_t*>(&quaternion), sizeof(quaternion),
      &quaternion_timestamp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get quaternion topic failed, error code: 0x%08X",
                   return_code);
  }

  return quaternion;
}

dji_f32_t GetLatestValueOfYawInDegree() {
  const T_DjiFcSubscriptionQuaternion current_quaternion =
      GetLatestValueOfQuaternion();
  const dji_f64_t yaw_current = std::atan2(
      2 * current_quaternion.q1 * current_quaternion.q2 +
          2 * current_quaternion.q0 * current_quaternion.q3,
      -2 * current_quaternion.q2 * current_quaternion.q2 -
          2 * current_quaternion.q3 * current_quaternion.q3 + 1);
  return static_cast<dji_f32_t>(yaw_current * kDegreesPerRadian);
}

}  // namespace

DjiGimbalController::DjiGimbalController(T_DjiOsalHandler* osal_handler)
    : initialized_(false),
      osal_handler_(osal_handler),
      node_handle_() {
  (void)osal_handler_;
  initialized_ = Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

  gimbal_pitch_angle_service_ = node_handle_.advertiseService(
      GetStringParam("/services/gimbal_pitch_angle_in_deg",
                     kDefaultGimbalPitchService),
      &DjiGimbalController::ServiceGimbalPitchAngleInDegCallback, this);
}

DjiGimbalController::~DjiGimbalController() {
  if (initialized_) {
    DeInit();
  }
}

T_DjiReturnCode DjiGimbalController::Init() {
  T_DjiReturnCode return_code = DjiGimbalManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init gimbal manager failed, error code: 0x%08X",
                   return_code);
    return return_code;
  }

  return_code = DjiGimbalManager_SetMode(kGimbalMountPosition,
                                         DJI_GIMBAL_MODE_YAW_FOLLOW);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Set gimbal mode failed, error code: 0x%08X", return_code);
    return return_code;
  }

  return_code = DjiGimbalManager_Reset(kGimbalMountPosition,
                                       DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", return_code);
    return return_code;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiGimbalController::DeInit() {
  const T_DjiReturnCode return_code = DjiGimbalManager_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit gimbal manager failed, error code: 0x%08X",
                   return_code);
  }
  return return_code;
}

bool DjiGimbalController::ServiceGimbalPitchAngleInDegCallback(
    indooruav_dji_driver::PSDK_GimbalPitchAngleInDeg::Request& request,
    indooruav_dji_driver::PSDK_GimbalPitchAngleInDeg::Response& response) {
  USER_LOG_INFO("------ServiceGimbalPitchAngleInDegCallback start------");

  if (!initialized_) {
    USER_LOG_ERROR("Gimbal controller is not initialized.");
    response.result = false;
    return true;
  }

  dji_f32_t pitch_angle = request.gimbal_pitch_angle_in_deg;
  if (pitch_angle > 0.0F) {
    pitch_angle = 0.0F;
  } else if (pitch_angle < -90.0F) {
    pitch_angle = -90.0F;
  }

  T_DjiGimbalManagerRotation rotation = {};
  rotation.pitch = pitch_angle;
  rotation.roll = 0.0F;
  rotation.yaw = GetLatestValueOfYawInDegree();
  rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
  rotation.time = 0.5;

  const T_DjiReturnCode return_code =
      DjiGimbalManager_Rotate(kGimbalMountPosition, rotation);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Rotate gimbal failed, error code: 0x%08X", return_code);
    response.result = false;
  } else {
    response.result = true;
  }

  USER_LOG_INFO("------ServiceGimbalPitchAngleInDegCallback end------");
  return true;
}
