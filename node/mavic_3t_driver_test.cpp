#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include "indooruav_dji_driver/PSDK_Landing.h"
#include "indooruav_dji_driver/PSDK_TakeOff.h"

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr double kArrivalToleranceMeters = 0.20;
constexpr double kCommandPublishFrequencyHz = 30.0;
constexpr double kWaitFrequencyHz = 10.0;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kXIndex = 0;
constexpr std::size_t kYIndex = 1;
constexpr std::size_t kZIndex = 2;
constexpr std::size_t kYawDegIndex = 3;

nav_msgs::Odometry g_current_odometry;
bool g_has_current_odometry = false;
std::mutex g_current_odometry_mutex;

std::string GetStringParam(
    const std::string& suffix,
    const std::string& default_value) {
  std::string value = default_value;
  ros::param::param<std::string>(
      std::string(kParamPrefix) + suffix, value, default_value);
  return value;
}

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

geometry_msgs::Quaternion CreateQuaternionFromYaw(double yaw_rad) {
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(yaw_rad * 0.5);
  quaternion.w = std::cos(yaw_rad * 0.5);
  return quaternion;
}

void CurrentOdometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(g_current_odometry_mutex);
  g_current_odometry = *message;
  g_has_current_odometry = true;
}

bool TryGetCurrentOdometry(nav_msgs::Odometry* odometry) {
  std::lock_guard<std::mutex> lock(g_current_odometry_mutex);
  if (!g_has_current_odometry) {
    return false;
  }

  *odometry = g_current_odometry;
  return true;
}

bool WaitForCurrentOdometry() {
  ros::Rate rate(kWaitFrequencyHz);
  while (ros::ok()) {
    nav_msgs::Odometry odometry;
    if (TryGetCurrentOdometry(&odometry)) {
      return true;
    }

    ROS_INFO_THROTTLE(1.0, "Waiting for current odometry...");
    rate.sleep();
  }

  return false;
}

bool WaitForService(ros::ServiceClient* client, const std::string& service_name) {
  while (ros::ok()) {
    if (client->waitForExistence(ros::Duration(1.0))) {
      return true;
    }

    ROS_WARN("Waiting for service: %s", service_name.c_str());
  }

  return false;
}

bool RequestTakeoff(ros::ServiceClient* takeoff_client) {
  indooruav_dji_driver::PSDK_TakeOff service;
  service.request.takeoff = true;

  if (!takeoff_client->call(service)) {
    ROS_ERROR("Failed to call takeoff service.");
    return false;
  }

  if (!service.response.result) {
    ROS_ERROR("Takeoff service returned failure.");
    return false;
  }

  ROS_INFO("Takeoff request succeeded.");
  return true;
}

bool RequestLanding(ros::ServiceClient* landing_client) {
  indooruav_dji_driver::PSDK_Landing service;
  service.request.landing = true;

  if (!landing_client->call(service)) {
    ROS_ERROR("Failed to call landing service.");
    return false;
  }

  if (!service.response.result) {
    ROS_ERROR("Landing service returned failure.");
    return false;
  }

  ROS_INFO("Landing request succeeded.");
  return true;
}

nav_msgs::Odometry BuildDesiredOdometry(
    const std::array<double, 4>& waypoint,
    const nav_msgs::Odometry& current_odometry) {
  nav_msgs::Odometry desired_odometry;
  desired_odometry.header.stamp = ros::Time::now();
  desired_odometry.header.frame_id = current_odometry.header.frame_id.empty()
                                         ? "world"
                                         : current_odometry.header.frame_id;
  desired_odometry.child_frame_id = current_odometry.child_frame_id;

  desired_odometry.pose.pose.position.x = waypoint[kXIndex];
  desired_odometry.pose.pose.position.y = waypoint[kYIndex];
  desired_odometry.pose.pose.position.z = waypoint[kZIndex];
  desired_odometry.pose.pose.orientation =
      CreateQuaternionFromYaw(DegreesToRadians(waypoint[kYawDegIndex]));

  return desired_odometry;
}

double ComputeDistanceToWaypoint(
    const nav_msgs::Odometry& current_odometry,
    const std::array<double, 4>& waypoint) {
  const double dx =
      current_odometry.pose.pose.position.x - waypoint[kXIndex];
  const double dy =
      current_odometry.pose.pose.position.y - waypoint[kYIndex];
  const double dz =
      current_odometry.pose.pose.position.z - waypoint[kZIndex];

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mavic_3t_driver_test");
  ros::NodeHandle node_handle;

  const std::vector<std::array<double, 4> > waypoints = {
      {0.0, 0.0, 1.0, 0.0},
      {0.5, 0.0, 1.5, 0.0},
      {1.0, 0.0, 1.5, 0.0},
      {1.5, 0.0, 1.5, 0.0},
      {2.0, 0.0, 1.5, 0.0},
      {2.5, 0.0, 1.5, 0.0},
  };

  const std::string desired_odometry_topic = GetStringParam(
      "/topics/desired_odometry", "/indooruav/desired_odometry");
  const std::string current_odometry_topic = GetStringParam(
      "/topics/current_odometry", "/indooruav/current_odometry");
  const std::string takeoff_service_name = GetStringParam(
      "/services/takeoff", "/PSDK/DjiFlightController/TakeOffService");
  const std::string landing_service_name = GetStringParam(
      "/services/landing", "/PSDK/DjiFlightController/LandingService");

  ros::Publisher desired_odometry_publisher =
      node_handle.advertise<nav_msgs::Odometry>(desired_odometry_topic, 10);
  ros::Subscriber current_odometry_subscriber =
      node_handle.subscribe(current_odometry_topic, 10, CurrentOdometryCallback);
  ros::ServiceClient takeoff_client =
      node_handle.serviceClient<indooruav_dji_driver::PSDK_TakeOff>(
          takeoff_service_name);
  ros::ServiceClient landing_client =
      node_handle.serviceClient<indooruav_dji_driver::PSDK_Landing>(
          landing_service_name);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (!WaitForService(&takeoff_client, takeoff_service_name)) {
    return 1;
  }

  if (!WaitForService(&landing_client, landing_service_name)) {
    return 1;
  }

  if (!WaitForCurrentOdometry()) {
    return 1;
  }

  if (!RequestTakeoff(&takeoff_client)) {
    return 1;
  }

  ros::Rate rate(kCommandPublishFrequencyHz);
  std::size_t waypoint_index = 0;
  while (ros::ok() && waypoint_index < waypoints.size()) {
    nav_msgs::Odometry current_odometry;
    if (!TryGetCurrentOdometry(&current_odometry)) {
      rate.sleep();
      continue;
    }

    desired_odometry_publisher.publish(
        BuildDesiredOdometry(waypoints[waypoint_index], current_odometry));

    const double distance =
        ComputeDistanceToWaypoint(current_odometry, waypoints[waypoint_index]);
    ROS_INFO_THROTTLE(
        1.0,
        "Flying to waypoint %lu/%lu, distance = %.2f m",
        static_cast<unsigned long>(waypoint_index + 1),
        static_cast<unsigned long>(waypoints.size()),
        distance);

    if (distance < kArrivalToleranceMeters) {
      ROS_INFO(
          "Reached waypoint %lu/%lu.",
          static_cast<unsigned long>(waypoint_index + 1),
          static_cast<unsigned long>(waypoints.size()));
      ++waypoint_index;
    }

    rate.sleep();
  }

  if (!ros::ok()) {
    return 0;
  }

  if (!RequestLanding(&landing_client)) {
    return 1;
  }

  current_odometry_subscriber.shutdown();
  spinner.stop();
  return 0;
}
