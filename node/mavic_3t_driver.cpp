#include <stdexcept>

#include <ros/ros.h>

#include <dji_platform.h>

#include "dependences/application.hpp"
#include "indooruav_dji_driver/mavic_3t_camera_controller.h"
#include "indooruav_dji_driver/mavic_3t_data_subscription_manager.h"
#include "indooruav_dji_driver/mavic_3t_flight_controller.h"
#include "indooruav_dji_driver/mavic_3t_gimbal_controller.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mavic_3t_driver");

  try {

    Application application(argc, argv);

    T_DjiOsalHandler* osal_handler = DjiPlatform_GetOsalHandler();
    if (osal_handler == nullptr) {
      ROS_FATAL("PSDK OSAL handler is unavailable after application startup.");
      return 1;
    }

    DjiDataSubscriptionManager data_subscription_manager;
    if (data_subscription_manager.CheckUsability()) {
      ROS_INFO("DJI data subscription manager initialized successfully.");
    } else {
      ROS_WARN("DJI data subscription manager is unavailable.");
    }

    DjiFlightController flight_controller(osal_handler);

    DjiGimbalController gimbal_controller(osal_handler);

    DjiCameraController camera_controller(osal_handler);

    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to start mavic_3t_driver: %s", exception.what());
    return 1;
  }

  return 0;
}
