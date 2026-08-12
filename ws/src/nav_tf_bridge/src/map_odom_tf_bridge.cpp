#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
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
    nav_tf_topic_ = declare_parameter<std::string>("nav_tf_topic", "/nav_tf");
    nav_base_frame_ = declare_parameter<std::string>("nav_base_frame", "base_link");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/relocalization_pose");
    initialpose_topic_ = declare_parameter<std::string>(
      "initialpose_topic", "/initialpose_relay");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    localization_to_base_roll_ = declare_parameter<double>(
      "localization_to_base_roll_rad", 0.007094763);
    localization_to_base_pitch_ = declare_parameter<double>(
      "localization_to_base_pitch_rad", -0.801751898);
    localization_to_base_yaw_ = declare_parameter<double>(
      "localization_to_base_yaw_rad", 1.570796327);
    localization_to_base_x_ = declare_parameter<double>(
      "localization_to_base_x_m", -0.115311164);
    localization_to_base_y_ = declare_parameter<double>(
      "localization_to_base_y_m", 0.468093861);
    localization_to_base_z_ = declare_parameter<double>(
      "localization_to_base_z_m", -0.592192935);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    odom_history_sec_ = declare_parameter<double>("odom_history_sec", 15.0);
    in_place_max_translation_ = declare_parameter<double>(
      "in_place_max_translation", 0.015);
    in_place_min_rotation_ = declare_parameter<double>(
      "in_place_min_rotation", 0.004);
    stationary_max_translation_ = declare_parameter<double>(
      "stationary_max_translation", 0.003);
    stationary_max_rotation_ = declare_parameter<double>(
      "stationary_max_rotation", 0.003);
    moving_max_translation_correction_ = declare_parameter<double>(
      "moving_max_translation_correction", 0.030);
    in_place_max_linear_speed_ = declare_parameter<double>(
      "in_place_max_linear_speed", 0.025);
    in_place_min_angular_speed_ = declare_parameter<double>(
      "in_place_min_angular_speed", 0.005);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    nav_tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(
      nav_tf_topic_, rclcpp::QoS(20).transient_local().reliable());
    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_pose_ = std::move(msg);
        pose_received_at_ = now();
      });
    initialpose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
        // A new operator-provided initial pose defines a discontinuous map
        // re-anchoring event.  The stationary-noise filter below must not
        // preserve the previous map->odom transform across that event, or a
        // cold-started RViz can show the last shutdown pose even though NDT
        // has already converged at the newly selected pose.
        latest_pose_.reset();
        have_processed_pose_ = false;
        have_cached_map_odom_ = false;
        have_processed_odom_ = false;
        have_publish_odom_ = false;
        in_place_anchor_active_ = false;
        RCLCPP_WARN(
          get_logger(),
          "initial pose received on %s; cleared the old map->odom anchor",
          initialpose_topic_.c_str());
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
      "bridging %s and %s into %s->%s at %.1f Hz and publishing "
      "direct %s->%s on %s; "
      "reset_topic=%s; "
      "localization->base xyz=(%.4f,%.4f,%.4f) rpy=(%.5f,%.5f,%.5f)",
      pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(),
      odom_frame_.c_str(), publish_rate_, map_frame_.c_str(),
      nav_base_frame_.c_str(), nav_tf_topic_.c_str(), initialpose_topic_.c_str(),
      localization_to_base_x_,
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
      tf2::Transform measured_map_base = map_localization * localization_base;
      if (have_cached_map_odom_ && have_processed_odom_) {
        const tf2::Transform odom_delta =
          processed_odom_base_.inverseTimes(odom_base);
        const double odom_translation = std::hypot(
          odom_delta.getOrigin().x(), odom_delta.getOrigin().y());
        const double odom_rotation = std::abs(
          odom_delta.getRotation().getAngleShortestPath());
        const tf2::Transform predicted_map_base = cached_map_odom_ * odom_base;

        const bool stationary =
          odom_translation <= stationary_max_translation_ &&
          odom_rotation <= stationary_max_rotation_;
        const bool rotating_in_place =
          odom_translation <= in_place_max_translation_ &&
          odom_rotation >= in_place_min_rotation_;

        if (stationary) {
          // Repeated NDT scans of an unmoving chassis contain centimetre-level
          // optimizer noise.  The short-term wheel pose is the stable source;
          // retain it until measured motion resumes.
          measured_map_base = predicted_map_base;
        } else if (rotating_in_place) {
          // During a physical in-place turn the base center must not orbit the
          // target because of LiDAR/NDT lever-arm noise.  Accept the measured
          // heading, but preserve the odometry-propagated chassis position.
          measured_map_base.setOrigin(predicted_map_base.getOrigin());
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "stabilizing in-place NDT translation (odom ds=%.4f m dyaw=%.3f deg)",
            odom_translation, odom_rotation * 180.0 / M_PI);
        } else {
          // Preserve long-term NDT correction during translation, while a
          // single optimizer sample cannot teleport the navigation frame.
          tf2::Vector3 correction =
            measured_map_base.getOrigin() - predicted_map_base.getOrigin();
          const double correction_length = correction.length();
          if (correction_length > moving_max_translation_correction_) {
            correction *= moving_max_translation_correction_ / correction_length;
            measured_map_base.setOrigin(
              predicted_map_base.getOrigin() + correction);
          }
        }
      }
      cached_map_base_ = measured_map_base;
      cached_map_odom_ = cached_map_base_ * odom_base.inverse();
      processed_odom_base_ = odom_base;
      have_processed_odom_ = true;
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

    tf2::Transform current_odom_base;
    tf2::fromMsg(latest_odom_->pose.pose, current_odom_base);
    const double odom_linear_speed = std::hypot(
      latest_odom_->twist.twist.linear.x,
      latest_odom_->twist.twist.linear.y);
    const double odom_angular_speed = std::abs(
      latest_odom_->twist.twist.angular.z);
    bool incremental_in_place_rotation = false;
    if (have_publish_odom_) {
      const tf2::Transform publish_delta =
        publish_odom_base_.inverseTimes(current_odom_base);
      const double publish_translation = std::hypot(
        publish_delta.getOrigin().x(), publish_delta.getOrigin().y());
      const double publish_rotation = std::abs(
        publish_delta.getRotation().getAngleShortestPath());
      incremental_in_place_rotation =
        publish_translation <= 0.005 && publish_rotation >= 0.0008;
    }
    const bool rotating_in_place_now =
      (odom_linear_speed <= in_place_max_linear_speed_ &&
      odom_angular_speed >= in_place_min_angular_speed_) ||
      incremental_in_place_rotation;

    if (rotating_in_place_now) {
      tf2::Transform current_map_base = cached_map_odom_ * current_odom_base;
      if (!in_place_anchor_active_) {
        in_place_anchor_ = current_map_base.getOrigin();
        in_place_anchor_active_ = true;
        RCLCPP_INFO(
          get_logger(),
          "locking map-frame chassis center for in-place rotation at "
          "(%.4f, %.4f)", in_place_anchor_.x(), in_place_anchor_.y());
      }
      current_map_base.setOrigin(in_place_anchor_);
      cached_map_base_ = current_map_base;
      cached_map_odom_ = current_map_base * current_odom_base.inverse();
    } else if (in_place_anchor_active_) {
      in_place_anchor_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "released map-frame chassis-center lock after in-place rotation");
    }
    publish_odom_base_ = current_odom_base;
    have_publish_odom_ = true;

    geometry_msgs::msg::TransformStamped output;
    output.header.stamp = now();
    output.header.frame_id = map_frame_;
    output.child_frame_id = odom_frame_;
    output.transform = tf2::toMsg(cached_map_odom_);
    broadcaster_->sendTransform(output);

    // Keep the reliable direct navigation pose synchronized with the actual
    // map->odom->base_link chain.  cached_map_base_ is only the chassis pose
    // at the last accepted localization sample; publishing it forever makes
    // RViz freeze while odometry and the physical chassis continue moving.
    const tf2::Transform current_map_base =
      cached_map_odom_ * current_odom_base;

    geometry_msgs::msg::TransformStamped nav_output;
    nav_output.header.stamp = output.header.stamp;
    nav_output.header.frame_id = map_frame_;
    nav_output.child_frame_id = nav_base_frame_;
    nav_output.transform = tf2::toMsg(current_map_base);
    tf2_msgs::msg::TFMessage nav_message;
    nav_message.transforms.push_back(std::move(nav_output));
    nav_tf_publisher_->publish(std::move(nav_message));
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string nav_tf_topic_;
  std::string nav_base_frame_;
  std::string pose_topic_;
  std::string initialpose_topic_;
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
  double in_place_max_translation_;
  double in_place_min_rotation_;
  double stationary_max_translation_;
  double stationary_max_rotation_;
  double moving_max_translation_correction_;
  double in_place_max_linear_speed_;
  double in_place_min_angular_speed_;
  struct TimedOdom
  {
    rclcpp::Time stamp;
    tf2::Transform transform;
  };
  std::deque<TimedOdom> odom_history_;
  tf2::Transform processed_map_localization_;
  tf2::Transform cached_map_odom_;
  tf2::Transform cached_map_base_;
  tf2::Transform processed_odom_base_;
  tf2::Transform publish_odom_base_;
  tf2::Vector3 in_place_anchor_;
  bool have_processed_pose_{false};
  bool have_cached_map_odom_{false};
  bool have_processed_odom_{false};
  bool have_publish_odom_{false};
  bool in_place_anchor_active_{false};
  rclcpp::Time pose_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_received_at_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr nav_tf_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfBridge>());
  rclcpp::shutdown();
  return 0;
}
