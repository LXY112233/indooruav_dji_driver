#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "indooruav_dji_driver/PSDK_Landing.h"
#include "indooruav_dji_driver/PSDK_TakeOff.h"

namespace {

constexpr char kParamPrefix[] = "/indooruav_dji_driver/dji_mavic_3t";
constexpr double kDefaultDesiredPublishRateHz = 30.0;
constexpr double kDefaultMissionLoopRateHz = 20.0;
constexpr double kDefaultWaypointToleranceM = 0.12;
constexpr double kDefaultWaypointHoldTimeS = 1.0;
constexpr double kDefaultOdomWaitTimeoutS = 30.0;
constexpr double kDefaultServiceWaitTimeoutS = 30.0;
constexpr double kDefaultTakeoffStabilizationS = 2.0;
constexpr char kDefaultDesiredOdometryTopic[] = "/indooruav/desired_odometry";
constexpr char kDefaultCurrentOdometryTopic[] = "/indooruav/current_odometry";
constexpr char kDefaultTakeoffService[] = "/PSDK/DjiFlightController/TakeOffService";
constexpr char kDefaultLandingService[] = "/PSDK/DjiFlightController/LandingService";
constexpr char kDefaultFrameId[] = "flu_world";

struct Waypoint {
  double x;
  double y;
  double z;
};

geometry_msgs::Quaternion CreateIdentityQuaternion() {
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;
  quaternion.w = 1.0;
  return quaternion;
}

std::vector<Waypoint> CreateMissionWaypoints() {
  return {
      {0.0, 0.0, 1.5},
      {0.3, 0.0, 1.5},
      {0.6, 0.0, 1.5},
      {0.9, 0.0, 1.5},
      {1.2, 0.0, 1.5},
  };
}

class Mavic3TPositionControlTest {
 public:
  Mavic3TPositionControlTest()
      : node_handle_(),
        private_node_handle_("~"),
        has_current_odometry_(false),
        desired_publishing_enabled_(false),
        desired_publish_rate_hz_(kDefaultDesiredPublishRateHz),
        mission_loop_rate_hz_(kDefaultMissionLoopRateHz),
        waypoint_tolerance_m_(kDefaultWaypointToleranceM),
        waypoint_hold_time_s_(kDefaultWaypointHoldTimeS),
        odom_wait_timeout_s_(kDefaultOdomWaitTimeoutS),
        service_wait_timeout_s_(kDefaultServiceWaitTimeoutS),
        takeoff_stabilization_s_(kDefaultTakeoffStabilizationS),
        desired_odometry_topic_(kDefaultDesiredOdometryTopic),
        current_odometry_topic_(kDefaultCurrentOdometryTopic),
        takeoff_service_name_(kDefaultTakeoffService),
        landing_service_name_(kDefaultLandingService),
        waypoints_(CreateMissionWaypoints()) {
    LoadConfig();

    current_odometry_subscriber_ = node_handle_.subscribe(
        current_odometry_topic_, 10,
        &Mavic3TPositionControlTest::CurrentOdometryCallback, this);
    desired_odometry_publisher_ =
        node_handle_.advertise<nav_msgs::Odometry>(desired_odometry_topic_, 10);

    takeoff_client_ = node_handle_.serviceClient<indooruav_dji_driver::PSDK_TakeOff>(
        takeoff_service_name_);
    landing_client_ = node_handle_.serviceClient<indooruav_dji_driver::PSDK_Landing>(
        landing_service_name_);

    desired_publish_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / desired_publish_rate_hz_),
        &Mavic3TPositionControlTest::DesiredPublishTimerCallback, this);
  }

  bool Run() {
    ros::AsyncSpinner spinner(2);
    spinner.start();

    if (!WaitForServices()) {
      return false;
    }

    if (!WaitForCurrentOdometry()) {
      return false;
    }

    ROS_INFO("Calling takeoff service: %s", takeoff_service_name_.c_str());
    if (!CallTakeoff()) {
      return false;
    }

    if (takeoff_stabilization_s_ > 0.0) {
      ROS_INFO("Waiting %.1f seconds for post-takeoff stabilization.",
               takeoff_stabilization_s_);
      ros::Duration(takeoff_stabilization_s_).sleep();
    }

    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      const Waypoint& waypoint = waypoints_[index];
      ROS_INFO(
          "Starting waypoint %lu/%lu: (%.2f, %.2f, %.2f) in FLU ground frame.",
          static_cast<unsigned long>(index + 1),
          static_cast<unsigned long>(waypoints_.size()),
          waypoint.x, waypoint.y, waypoint.z);

      SetActiveWaypoint(waypoint);
      if (!WaitUntilWaypointReached(waypoint, index)) {
        StopDesiredPublishing();
        AttemptLandingOnFailure();
        return false;
      }
    }

    StopDesiredPublishing();
    ROS_INFO("All waypoints reached. Calling landing service: %s",
             landing_service_name_.c_str());

    if (!CallLanding()) {
      return false;
    }

    ROS_INFO("Mission completed. Landing command sent successfully.");
    return true;
  }

 private:
  void LoadConfig() {
    const std::string prefix(kParamPrefix);

    ros::param::param<std::string>(
        prefix + "/topics/desired_odometry", desired_odometry_topic_,
        desired_odometry_topic_);
    ros::param::param<std::string>(
        prefix + "/topics/current_odometry", current_odometry_topic_,
        current_odometry_topic_);
    ros::param::param<std::string>(
        prefix + "/services/takeoff", takeoff_service_name_,
        takeoff_service_name_);
    ros::param::param<std::string>(
        prefix + "/services/landing", landing_service_name_,
        landing_service_name_);

    private_node_handle_.param(
        "desired_publish_rate_hz", desired_publish_rate_hz_,
        desired_publish_rate_hz_);
    private_node_handle_.param(
        "mission_loop_rate_hz", mission_loop_rate_hz_, mission_loop_rate_hz_);
    private_node_handle_.param(
        "waypoint_tolerance_m", waypoint_tolerance_m_, waypoint_tolerance_m_);
    private_node_handle_.param(
        "waypoint_hold_time_s", waypoint_hold_time_s_, waypoint_hold_time_s_);
    private_node_handle_.param(
        "odom_wait_timeout_s", odom_wait_timeout_s_, odom_wait_timeout_s_);
    private_node_handle_.param(
        "service_wait_timeout_s", service_wait_timeout_s_,
        service_wait_timeout_s_);
    private_node_handle_.param(
        "takeoff_stabilization_s", takeoff_stabilization_s_,
        takeoff_stabilization_s_);

    if (desired_publish_rate_hz_ <= 0.0) {
      ROS_WARN("desired_publish_rate_hz must be > 0. Resetting to %.1f Hz.",
               kDefaultDesiredPublishRateHz);
      desired_publish_rate_hz_ = kDefaultDesiredPublishRateHz;
    }

    if (mission_loop_rate_hz_ <= 0.0) {
      ROS_WARN("mission_loop_rate_hz must be > 0. Resetting to %.1f Hz.",
               kDefaultMissionLoopRateHz);
      mission_loop_rate_hz_ = kDefaultMissionLoopRateHz;
    }

    if (waypoint_tolerance_m_ <= 0.0) {
      ROS_WARN("waypoint_tolerance_m must be > 0. Resetting to %.2f m.",
               kDefaultWaypointToleranceM);
      waypoint_tolerance_m_ = kDefaultWaypointToleranceM;
    }

    if (waypoint_hold_time_s_ < 0.0) {
      ROS_WARN("waypoint_hold_time_s must be >= 0. Resetting to %.1f s.",
               kDefaultWaypointHoldTimeS);
      waypoint_hold_time_s_ = kDefaultWaypointHoldTimeS;
    }
  }

  void CurrentOdometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_current_odometry_ = *message;
    has_current_odometry_ = true;
  }

  void DesiredPublishTimerCallback(const ros::TimerEvent& event) {
    (void)event;

    Waypoint waypoint = {};
    nav_msgs::Odometry current_odometry;
    bool has_current_odometry = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!desired_publishing_enabled_) {
        return;
      }

      waypoint = active_waypoint_;
      current_odometry = latest_current_odometry_;
      has_current_odometry = has_current_odometry_;
    }

    nav_msgs::Odometry desired_odometry;
    desired_odometry.header.stamp =
        has_current_odometry && !current_odometry.header.stamp.isZero()
            ? current_odometry.header.stamp
            : ros::Time::now();
    desired_odometry.header.frame_id =
        has_current_odometry && !current_odometry.header.frame_id.empty()
            ? current_odometry.header.frame_id
            : kDefaultFrameId;
    desired_odometry.child_frame_id =
        has_current_odometry ? current_odometry.child_frame_id : std::string();

    desired_odometry.pose.pose.position.x = waypoint.x;
    desired_odometry.pose.pose.position.y = waypoint.y;
    desired_odometry.pose.pose.position.z = waypoint.z;

    // Keep the commanded attitude aligned with the latest measured attitude so
    // this test focuses on position translation rather than yaw control.
    desired_odometry.pose.pose.orientation =
        has_current_odometry ? current_odometry.pose.pose.orientation
                             : CreateIdentityQuaternion();

    desired_odometry_publisher_.publish(desired_odometry);
  }

  bool WaitForServices() {
    const ros::Duration timeout(service_wait_timeout_s_);

    ROS_INFO("Waiting for takeoff service: %s", takeoff_service_name_.c_str());
    if (!takeoff_client_.waitForExistence(timeout)) {
      ROS_ERROR("Timed out waiting for takeoff service: %s",
                takeoff_service_name_.c_str());
      return false;
    }

    ROS_INFO("Waiting for landing service: %s", landing_service_name_.c_str());
    if (!landing_client_.waitForExistence(timeout)) {
      ROS_ERROR("Timed out waiting for landing service: %s",
                landing_service_name_.c_str());
      return false;
    }

    return true;
  }

  bool WaitForCurrentOdometry() {
    ROS_INFO("Waiting for current odometry on topic: %s",
             current_odometry_topic_.c_str());

    const ros::Time start_time = ros::Time::now();
    ros::Rate rate(mission_loop_rate_hz_);

    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_current_odometry_) {
          ROS_INFO("Current odometry received.");
          return true;
        }
      }

      if (odom_wait_timeout_s_ > 0.0 &&
          (ros::Time::now() - start_time).toSec() > odom_wait_timeout_s_) {
        ROS_ERROR("Timed out waiting for current odometry.");
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  void SetActiveWaypoint(const Waypoint& waypoint) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_waypoint_ = waypoint;
    desired_publishing_enabled_ = true;
  }

  void StopDesiredPublishing() {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_publishing_enabled_ = false;
  }

  bool GetLatestCurrentOdometry(nav_msgs::Odometry* odometry) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_current_odometry_) {
      return false;
    }

    *odometry = latest_current_odometry_;
    return true;
  }

  bool WaitUntilWaypointReached(const Waypoint& waypoint, std::size_t index) {
    ros::Rate rate(mission_loop_rate_hz_);
    ros::Time within_tolerance_since;
    bool within_tolerance = false;

    while (ros::ok()) {
      nav_msgs::Odometry current_odometry;
      if (!GetLatestCurrentOdometry(&current_odometry)) {
        ROS_WARN_THROTTLE(1.0, "Current odometry has not been received yet.");
        rate.sleep();
        continue;
      }

      const double dx = waypoint.x - current_odometry.pose.pose.position.x;
      const double dy = waypoint.y - current_odometry.pose.pose.position.y;
      const double dz = waypoint.z - current_odometry.pose.pose.position.z;

      const bool position_reached =
          std::fabs(dx) <= waypoint_tolerance_m_ &&
          std::fabs(dy) <= waypoint_tolerance_m_ &&
          std::fabs(dz) <= waypoint_tolerance_m_;

      if (position_reached) {
        if (!within_tolerance) {
          within_tolerance = true;
          within_tolerance_since = ros::Time::now();
          ROS_INFO(
              "Waypoint %lu entered tolerance window. position error = (%.3f, %.3f, %.3f)",
              static_cast<unsigned long>(index + 1), dx, dy, dz);
        }

        if ((ros::Time::now() - within_tolerance_since).toSec() >=
            waypoint_hold_time_s_) {
          ROS_INFO("Waypoint %lu confirmed.",
                   static_cast<unsigned long>(index + 1));
          return true;
        }
      } else {
        within_tolerance = false;
        ROS_INFO_THROTTLE(
            1.0,
            "Heading to waypoint %lu: current = (%.3f, %.3f, %.3f), error = (%.3f, %.3f, %.3f)",
            static_cast<unsigned long>(index + 1),
            current_odometry.pose.pose.position.x,
            current_odometry.pose.pose.position.y,
            current_odometry.pose.pose.position.z, dx, dy, dz);
      }

      rate.sleep();
    }

    return false;
  }

  bool CallTakeoff() {
    indooruav_dji_driver::PSDK_TakeOff service;
    service.request.takeoff = true;

    if (!takeoff_client_.call(service)) {
      ROS_ERROR("Failed to call takeoff service: %s",
                takeoff_service_name_.c_str());
      return false;
    }

    if (!service.response.result) {
      ROS_ERROR("Takeoff service returned false.");
      return false;
    }

    ROS_INFO("Takeoff command completed successfully.");
    return true;
  }

  bool CallLanding() {
    indooruav_dji_driver::PSDK_Landing service;
    service.request.landing = true;

    if (!landing_client_.call(service)) {
      ROS_ERROR("Failed to call landing service: %s",
                landing_service_name_.c_str());
      return false;
    }

    if (!service.response.result) {
      ROS_ERROR("Landing service returned false.");
      return false;
    }

    ROS_INFO("Landing command completed successfully.");
    return true;
  }

  void AttemptLandingOnFailure() {
    ROS_WARN("Mission failed after takeoff. Attempting to send landing command.");
    CallLanding();
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber current_odometry_subscriber_;
  ros::Publisher desired_odometry_publisher_;
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient landing_client_;
  ros::Timer desired_publish_timer_;

  std::mutex mutex_;
  nav_msgs::Odometry latest_current_odometry_;
  Waypoint active_waypoint_;
  bool has_current_odometry_;
  bool desired_publishing_enabled_;

  double desired_publish_rate_hz_;
  double mission_loop_rate_hz_;
  double waypoint_tolerance_m_;
  double waypoint_hold_time_s_;
  double odom_wait_timeout_s_;
  double service_wait_timeout_s_;
  double takeoff_stabilization_s_;

  std::string desired_odometry_topic_;
  std::string current_odometry_topic_;
  std::string takeoff_service_name_;
  std::string landing_service_name_;
  std::vector<Waypoint> waypoints_;
};

}  // namespace

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mavic_3t_driver_test");

  Mavic3TPositionControlTest test_node;
  return test_node.Run() ? 0 : 1;
}
