#include "indooruav_dji_driver/mavic_3t_camera_controller.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <dji_error.h>
#include <dji_logger.h>

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr char kDefaultShootPhotoService[] = "/PSDK/DjiCameraController/CameraShootPhoto";
constexpr E_DjiMountPosition kCameraMountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
constexpr float kMinOpticalZoomFactor = 1.0F; //最小变焦倍数
constexpr float kMaxOpticalZoomFactor = 6.9F; //最大变焦倍数（因为是拍摄近处目标，变焦倍数过大一定是模糊的）
constexpr float kZoomTolerance = 0.01F;

//从yaml里获取参数
std::string GetStringParam(const std::string& suffix,
                           const std::string& default_value) {
  std::string value = default_value;
  ros::param::param<std::string>(std::string(kParamPrefix) + suffix, value, default_value);
  return value;
}

}  // namespace

DjiCameraController::DjiCameraController(T_DjiOsalHandler* osal_handler)
    : initialized_(false), osal_handler_(osal_handler), node_handle_() 
{
  initialized_ = Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

  shoot_photo_server_ = node_handle_.advertiseService(
      GetStringParam("/services/camera_shoot_photo", kDefaultShootPhotoService),
      &DjiCameraController::ServiceCameraShootPhotoCallback, this);
}

DjiCameraController::~DjiCameraController() {
  if (initialized_) {
    DeInit();
  }
}

T_DjiReturnCode DjiCameraController::Init() {
  const T_DjiReturnCode return_code = DjiCameraManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X", return_code);
  }
  return return_code;
}

T_DjiReturnCode DjiCameraController::DeInit() {
  const T_DjiReturnCode return_code = DjiCameraManager_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Camera manager deinit failed, error code: 0x%08X", return_code);
  }
  return return_code;
}

bool DjiCameraController::ShootPhoto() {
  if (!initialized_) {
    USER_LOG_ERROR("Camera controller is not initialized.");
    return false;
  }

  bool connect_status = false;
  T_DjiReturnCode return_code = DjiCameraManager_GetCameraConnectStatus(kCameraMountPosition, &connect_status);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || !connect_status) {
    USER_LOG_ERROR("Camera is not connected, error code: 0x%08X", return_code);
    return false;
  }

  return_code = DjiCameraManager_SetMode(kCameraMountPosition, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Set camera shoot-photo mode failed, error code: 0x%08X", return_code);
    return false;
  }

  return_code = DjiCameraManager_SetShootPhotoMode(kCameraMountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Set single-shot mode failed, error code: 0x%08X", return_code);
    return false;
  }

  return_code = DjiCameraManager_StartShootPhoto(kCameraMountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Start shooting photo failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiCameraController::SetOpticalZoomParam(dji_f32_t zoom_factor) {
  if (!initialized_) {
    USER_LOG_ERROR("Camera controller is not initialized.");
    return false;
  }

  T_DjiCameraManagerOpticalZoomParam zoom_param = {};
  T_DjiReturnCode return_code = DjiCameraManager_GetOpticalZoomParam(kCameraMountPosition, &zoom_param);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get optical zoom param failed, error code: 0x%08X", return_code);
    return false;
  }

  dji_f32_t target_zoom = zoom_factor;
  if (target_zoom < kMinOpticalZoomFactor) {
    target_zoom = kMinOpticalZoomFactor;
  } else if (target_zoom > kMaxOpticalZoomFactor) {
    target_zoom = kMaxOpticalZoomFactor;
  }

  if (std::fabs(target_zoom - zoom_param.currentOpticalZoomFactor) < kZoomTolerance) {
    return true;
  }

  const E_DjiCameraZoomDirection zoom_direction = target_zoom > zoom_param.currentOpticalZoomFactor ? DJI_CAMERA_ZOOM_DIRECTION_IN : DJI_CAMERA_ZOOM_DIRECTION_OUT;

  return_code = DjiCameraManager_SetOpticalZoomParam(kCameraMountPosition, zoom_direction, target_zoom);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Set optical zoom param failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiCameraController::StartRecord() {
  if (!initialized_) {
    USER_LOG_ERROR("Camera controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiCameraManager_StartRecordVideo(kCameraMountPosition);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Start record failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiCameraController::StopRecord() {
  if (!initialized_) {
    USER_LOG_ERROR("Camera controller is not initialized.");
    return false;
  }

  const T_DjiReturnCode return_code = DjiCameraManager_StopRecordVideo(kCameraMountPosition);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Stop record failed, error code: 0x%08X", return_code);
    return false;
  }

  return true;
}

bool DjiCameraController::ServiceCameraShootPhotoCallback(
    indooruav_dji_driver::PSDK_CameraShootPhoto::Request& request,
    indooruav_dji_driver::PSDK_CameraShootPhoto::Response& response) 
{
  USER_LOG_INFO("------ServiceCameraShootPhotoCallback start------");

  if (!request.shoot_photo) {
    USER_LOG_ERROR("Received false shoot_photo request, nothing to do.");
    response.result = false;
    return true;
  }

  if (request.zoom_factor >= kMinOpticalZoomFactor && !SetOpticalZoomParam(request.zoom_factor)) {
    response.result = false;
    return true;
  }

  response.result = ShootPhoto();
  USER_LOG_INFO("------ServiceCameraShootPhotoCallback end------");
  return true;
}
