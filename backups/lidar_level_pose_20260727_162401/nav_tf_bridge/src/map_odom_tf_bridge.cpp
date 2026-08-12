#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MapOdomTfBridge : public rclcpp::Node
{
public:
  MapOdomTfBridge()
  : Node("map_odom_tf_bridge_cpp")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/relocalization_pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    localization_to_base_roll_ = declare_parameter<double>(
      "localization_to_base_roll_rad", -0.010669779);
    localization_to_base_pitch_ = declare_parameter<double>(
      "localization_to_base_pitch_rad", -0.274581016);
    localization_to_base_yaw_ = declare_parameter<double>(
      "localization_to_base_yaw_rad", -1.570796327);
    localization_to_base_x_ = declare_parameter<double>(
      "localization_to_base_x_m", 0.042528450);
    localization_to_base_y_ = declare_parameter<double>(
      "localization_to_base_y_m", 0.666725193);
    localization_to_base_z_ = declare_parameter<double>(
      "localization_to_base_z_m", -0.915570231);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    odom_history_sec_ = declare_parameter<double>("odom_history_sec", 15.0);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_pose_ = std::move(msg);
        pose_received_at_ = now();
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = std::move(msg);
        odom_received_at_ = now();
        tf2::Transform odom_base;
        tf2::fromMsg(latest_odom_->pose.pose, odom_base);
        rclcpp::Time stamp(latest_odom_->header.stamp);
        if (stamp.nanoseconds() == 0) {
          stamp = odom_received_at_;
        }
        odom_history_.push_back({stamp, odom_base});
        while (!odom_history_.empty() &&
          (stamp - odom_history_.front().stamp).seconds() > odom_history_sec_)
        {
          odom_history_.pop_front();
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapOdomTfBridge::publish_transform, this));
    RCLCPP_INFO(
      get_logger(),
      "bridging %s and %s into %s->%s at %.1f Hz; "
      "localization->base xyz=(%.4f,%.4f,%.4f) rpy=(%.5f,%.5f,%.5f)",
      pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(),
      odom_frame_.c_str(), publish_rate_, localization_to_base_x_,
      localization_to_base_y_, localization_to_base_z_,
      localization_to_base_roll_, localization_to_base_pitch_,
      localization_to_base_yaw_);
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() > 0 && (now() - stamp).seconds() <= input_timeout_;
  }

  void publish_transform()
  {
    if (!latest_pose_ || !latest_odom_ ||
      (now() - odom_received_at_).seconds() > odom_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for fresh %s and %s", pose_topic_.c_str(), odom_topic_.c_str());
      return;
    }

    const bool localization_fresh = fresh(pose_received_at_);
    if (!localization_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "localization is stale; holding the last map->odom correction");
    }

    tf2::Transform map_localization;
    tf2::fromMsg(latest_pose_->pose, map_localization);
    const bool pose_changed = localization_fresh && (!have_processed_pose_ ||
      (map_localization.getOrigin() - processed_map_localization_.getOrigin()).length() > 1.0e-4 ||
      std::abs(map_localization.getRotation().angleShortestPath(
        processed_map_localization_.getRotation())) > 1.0e-4);

    if (pose_changed) {
      tf2::Quaternion mount_rotation;
      mount_rotation.setRPY(
        localization_to_base_roll_, localization_to_base_pitch_,
        localization_to_base_yaw_);
      const tf2::Transform localization_base(
        mount_rotation,
        tf2::Vector3(
          localization_to_base_x_, localization_to_base_y_, localization_to_base_z_));

      const rclcpp::Time pose_stamp(latest_pose_->header.stamp);
      const TimedOdom * closest = nullptr;
      double closest_dt = std::numeric_limits<double>::infinity();
      for (const auto & sample : odom_history_) {
        const double dt = std::abs((sample.stamp - pose_stamp).seconds());
        if (dt < closest_dt) {
          closest = &sample;
          closest_dt = dt;
        }
      }
      tf2::Transform odom_base;
      if (closest != nullptr && pose_stamp.nanoseconds() > 0) {
        odom_base = closest->transform;
      } else {
        tf2::fromMsg(latest_odom_->pose.pose, odom_base);
      }
      cached_map_odom_ =
        map_localization * localization_base * odom_base.inverse();
      processed_map_localization_ = map_localization;
      have_processed_pose_ = true;
      have_cached_map_odom_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "updated map->odom from new localization pose (odom dt=%.3f s)",
        std::isfinite(closest_dt) ? closest_dt : -1.0);
    }
    if (!have_cached_map_odom_) {
      return;
    }

    geometry_msgs::msg::TransformStamped output;
    output.header.stamp = now();
    output.header.frame_id = map_frame_;
    output.child_frame_id = odom_frame_;
    output.transform = tf2::toMsg(cached_map_odom_);
    broadcaster_->sendTransform(output);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string pose_topic_;
  std::string odom_topic_;
  double localization_to_base_roll_;
  double localization_to_base_pitch_;
  double localization_to_base_yaw_;
  double localization_to_base_x_;
  double localization_to_base_y_;
  double localization_to_base_z_;
  double publish_rate_;
  double input_timeout_;
  double odom_timeout_;
  double odom_history_sec_;
  struct TimedOdom
  {
    rclcpp::Time stamp;
    tf2::Transform transform;
  };
  std::deque<TimedOdom> odom_history_;
  tf2::Transform processed_map_localization_;
  tf2::Transform cached_map_odom_;
  bool have_processed_pose_{false};
  bool have_cached_map_odom_{false};
  rclcpp::Time pose_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_received_at_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfBridge>());
  rclcpp::shutdown();
  return 0;
}
