#ifndef INDOORUAV_DJI_DRIVER_MAVIC_3T_CAMERA_CONTROLLER_H_
#define INDOORUAV_DJI_DRIVER_MAVIC_3T_CAMERA_CONTROLLER_H_

#include <ros/ros.h>

#include <dji_camera_manager.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include "indooruav_dji_driver/PSDK_CameraShootPhoto.h"

class DjiCameraController {
 public:
  explicit DjiCameraController(T_DjiOsalHandler* osal_handler);
  DjiCameraController() = delete;
  DjiCameraController(const DjiCameraController& other) = delete;
  DjiCameraController& operator=(const DjiCameraController& other) = delete;
  ~DjiCameraController();

 private:
  T_DjiReturnCode Init();
  T_DjiReturnCode DeInit();

  bool ShootPhoto();
  bool SetOpticalZoomParam(dji_f32_t zoom_factor);
  bool StartRecord();   //未使用
  bool StopRecord();    //未使用
  bool ServiceCameraShootPhotoCallback(
      indooruav_dji_driver::PSDK_CameraShootPhoto::Request& request,
      indooruav_dji_driver::PSDK_CameraShootPhoto::Response& response);

  bool initialized_;
  T_DjiOsalHandler* osal_handler_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer shoot_photo_server_;
};

#endif  // INDOORUAV_DJI_DRIVER_MAVIC_3T_CAMERA_CONTROLLER_H_
