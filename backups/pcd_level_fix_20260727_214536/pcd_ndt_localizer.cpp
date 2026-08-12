#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Eigen::Matrix4f odomToMatrix(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  Eigen::Quaternionf quat(
    static_cast<float>(q.w), static_cast<float>(q.x),
    static_cast<float>(q.y), static_cast<float>(q.z));
  quat.normalize();

  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = quat.toRotationMatrix();
  out(0, 3) = static_cast<float>(p.x);
  out(1, 3) = static_cast<float>(p.y);
  out(2, 3) = static_cast<float>(p.z);
  return out;
}

Eigen::Matrix4f poseWithCovToMatrix(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  Eigen::Quaternionf quat(
    static_cast<float>(q.w), static_cast<float>(q.x),
    static_cast<float>(q.y), static_cast<float>(q.z));
  quat.normalize();

  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = quat.toRotationMatrix();
  out(0, 3) = static_cast<float>(p.x);
  out(1, 3) = static_cast<float>(p.y);
  out(2, 3) = static_cast<float>(p.z);
  return out;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f & mat)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Matrix3f rot = mat.block<3, 3>(0, 0);
  Eigen::Quaternionf quat(rot);
  quat.normalize();
  pose.position.x = mat(0, 3);
  pose.position.y = mat(1, 3);
  pose.position.z = mat(2, 3);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

CloudT::Ptr voxelDownsample(const CloudT::ConstPtr & cloud, double leaf_size)
{
  if (leaf_size <= 0.0) {
    return CloudT::Ptr(new CloudT(*cloud));
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(cloud);
  CloudT::Ptr filtered(new CloudT);
  voxel.filter(*filtered);
  return filtered;
}
}  // namespace

class PcdNdtLocalizer : public rclcpp::Node
{
public:
  PcdNdtLocalizer() : Node("pcd_ndt_localizer")
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered_body");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");
    localization_to_base_roll_rad_ =
      declare_parameter<double>("localization_to_base_roll_rad", 0.0);
    localization_to_base_pitch_rad_ =
      declare_parameter<double>("localization_to_base_pitch_rad", 0.0);
    localization_to_base_yaw_rad_ =
      declare_parameter<double>("localization_to_base_yaw_rad", 0.0);
    localization_to_base_x_m_ =
      declare_parameter<double>("localization_to_base_x_m", 0.0);
    localization_to_base_y_m_ =
      declare_parameter<double>("localization_to_base_y_m", 0.0);
    localization_to_base_z_m_ =
      declare_parameter<double>("localization_to_base_z_m", 0.0);
    visualization_yaw_offset_rad_ =
      declare_parameter<double>("visualization_yaw_offset_rad", 0.0);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.0);
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.5);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.25);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 80);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.1);
    ndt_trans_eps_ = declare_parameter<double>("ndt_trans_eps", 0.01);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 35);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 3.0);
    path_keep_count_ = declare_parameter<int>("path_keep_count", 300);
    require_initial_pose_ = declare_parameter<bool>("require_initial_pose", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_map_cloud_ = declare_parameter<bool>("publish_map_cloud", true);
    publish_aligned_cloud_ = declare_parameter<bool>("publish_aligned_cloud", true);
    map_publish_period_sec_ = declare_parameter<double>("map_publish_period_sec", 2.0);
    publish_occupancy_grid_ = declare_parameter<bool>("publish_occupancy_grid", false);
    occupancy_resolution_ = declare_parameter<double>("occupancy_resolution", 0.10);
    occupancy_min_z_ = declare_parameter<double>("occupancy_min_z", -0.2);
    occupancy_max_z_ = declare_parameter<double>("occupancy_max_z", 2.0);
    occupancy_padding_m_ = declare_parameter<double>("occupancy_padding_m", 1.0);
    occupancy_inflate_cells_ = declare_parameter<int>("occupancy_inflate_cells", 1);
    occupancy_min_points_per_cell_ = declare_parameter<int>("occupancy_min_points_per_cell", 3);
    occupancy_min_z_span_ = declare_parameter<double>("occupancy_min_z_span", 0.15);
    occupancy_save_map_files_ = declare_parameter<bool>("occupancy_save_map_files", false);
    occupancy_yaml_path_ = declare_parameter<std::string>("occupancy_yaml_path", "");
    reject_occupied_pose_ = declare_parameter<bool>("reject_occupied_pose", true);
    reject_pose_outside_grid_ = declare_parameter<bool>("reject_pose_outside_grid", true);
    occupancy_check_radius_cells_ = declare_parameter<int>("occupancy_check_radius_cells", 1);
    occupancy_reject_min_occupied_cells_ =
      declare_parameter<int>("occupancy_reject_min_occupied_cells", 1);

    loadMap();

    ndt_.setTransformationEpsilon(ndt_trans_eps_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
    ndt_.setInputTarget(map_cloud_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/relocalization_pose", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/relocalization_odom", rclcpp::SensorDataQoS());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/relocalization_path", rclcpp::SensorDataQoS());
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/aligned_cloud", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/relocalization_markers", rclcpp::QoS(1).transient_local().reliable());
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", rclcpp::QoS(1).transient_local().reliable());
    if (publish_occupancy_grid_) {
      occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/pcd_occupancy_grid", rclcpp::QoS(1).transient_local().reliable());
      boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcd_boundary_cloud", rclcpp::QoS(1).transient_local().reliable());
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdNdtLocalizer::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdNdtLocalizer::cloudCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 10,
      std::bind(&PcdNdtLocalizer::initialPoseCallback, this, std::placeholders::_1));

    if (publish_map_cloud_ || publish_occupancy_grid_) {
      map_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(map_publish_period_sec_)),
        std::bind(&PcdNdtLocalizer::publishMap, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "PCD NDT localization ready. map=%s points=%zu require_initial_pose=%d reject_occupied_pose=%d",
      map_path_.c_str(), map_cloud_->size(), require_initial_pose_, reject_occupied_pose_);
  }

private:
  Eigen::Matrix4f localizationToBase() const
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) =
      (Eigen::AngleAxisf(
        static_cast<float>(localization_to_base_yaw_rad_),
        Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(
        static_cast<float>(localization_to_base_pitch_rad_),
        Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(
        static_cast<float>(localization_to_base_roll_rad_),
        Eigen::Vector3f::UnitX())).toRotationMatrix();
    transform(0, 3) = static_cast<float>(localization_to_base_x_m_);
    transform(1, 3) = static_cast<float>(localization_to_base_y_m_);
    transform(2, 3) = static_cast<float>(localization_to_base_z_m_);
    return transform;
  }

  void loadMap()
  {
    if (map_path_.empty()) {
      throw std::runtime_error("map_path is empty");
    }
    CloudT::Ptr raw(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw) != 0) {
      throw std::runtime_error("failed to load PCD map: " + map_path_);
    }
    map_cloud_ = voxelDownsample(raw, map_leaf_size_);
    map_cloud_->header.frame_id = global_frame_;
    RCLCPP_INFO(get_logger(), "Loaded map %s raw=%zu filtered=%zu leaf=%.3f",
      map_path_.c_str(), raw->size(), map_cloud_->size(), map_leaf_size_);

    if (publish_occupancy_grid_ || reject_occupied_pose_) {
      buildOccupancyGrid(raw);
    }
  }

  std::size_t gridIndex(int x, int y) const
  {
    return static_cast<std::size_t>(y) * occupancy_width_ + static_cast<std::size_t>(x);
  }

  std::vector<std::int8_t> inflateGrid(const std::vector<std::int8_t> & input) const
  {
    if (occupancy_inflate_cells_ <= 0) {
      return input;
    }

    std::vector<std::int8_t> output = input;
    for (int y = 0; y < static_cast<int>(occupancy_height_); ++y) {
      for (int x = 0; x < static_cast<int>(occupancy_width_); ++x) {
        if (input[gridIndex(x, y)] != 100) {
          continue;
        }
        for (int dy = -occupancy_inflate_cells_; dy <= occupancy_inflate_cells_; ++dy) {
          for (int dx = -occupancy_inflate_cells_; dx <= occupancy_inflate_cells_; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && ny >= 0 &&
                nx < static_cast<int>(occupancy_width_) &&
                ny < static_cast<int>(occupancy_height_)) {
              output[gridIndex(nx, ny)] = 100;
            }
          }
        }
      }
    }
    return output;
  }

  void buildOccupancyGrid(const CloudT::ConstPtr & raw)
  {
    if (occupancy_resolution_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "occupancy_resolution must be positive; skip occupancy grid");
      publish_occupancy_grid_ = false;
      return;
    }
    if (occupancy_min_z_ >= occupancy_max_z_) {
      RCLCPP_WARN(get_logger(), "occupancy_min_z must be smaller than occupancy_max_z; skip occupancy grid");
      publish_occupancy_grid_ = false;
      return;
    }

    std::vector<PointT> kept;
    kept.reserve(raw->size());
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto & point : raw->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
          point.z < occupancy_min_z_ || point.z > occupancy_max_z_) {
        continue;
      }
      kept.push_back(point);
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    if (kept.empty()) {
      RCLCPP_WARN(get_logger(), "no points kept for occupancy grid; adjust occupancy_min_z/max_z");
      publish_occupancy_grid_ = false;
      return;
    }

    occupancy_origin_x_ = min_x - occupancy_padding_m_;
    occupancy_origin_y_ = min_y - occupancy_padding_m_;
    const double span_x = max_x - min_x + 2.0 * occupancy_padding_m_;
    const double span_y = max_y - min_y + 2.0 * occupancy_padding_m_;
    occupancy_width_ = static_cast<std::uint32_t>(std::ceil(span_x / occupancy_resolution_));
    occupancy_height_ = static_cast<std::uint32_t>(std::ceil(span_y / occupancy_resolution_));

    struct CellStats
    {
      int count = 0;
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
    };

    std::vector<CellStats> stats(occupancy_width_ * occupancy_height_);
    for (const auto & point : kept) {
      const int x = static_cast<int>(std::floor((point.x - occupancy_origin_x_) / occupancy_resolution_));
      const int y = static_cast<int>(std::floor((point.y - occupancy_origin_y_) / occupancy_resolution_));
      if (x >= 0 && y >= 0 &&
          x < static_cast<int>(occupancy_width_) &&
          y < static_cast<int>(occupancy_height_)) {
        auto & cell = stats[gridIndex(x, y)];
        ++cell.count;
        cell.min_z = std::min(cell.min_z, point.z);
        cell.max_z = std::max(cell.max_z, point.z);
      }
    }

    std::vector<std::int8_t> occupied(occupancy_width_ * occupancy_height_, 0);
    const int min_points = std::max(1, occupancy_min_points_per_cell_);
    std::size_t occupied_cells = 0;
    for (std::size_t i = 0; i < stats.size(); ++i) {
      const auto & cell = stats[i];
      if (cell.count < min_points) {
        continue;
      }
      const float z_span = cell.max_z - cell.min_z;
      if (occupancy_min_z_span_ > 0.0 &&
          z_span < static_cast<float>(occupancy_min_z_span_)) {
        continue;
      }
      occupied[i] = 100;
      ++occupied_cells;
    }

    occupancy_msg_.header.frame_id = global_frame_;
    occupancy_msg_.info.resolution = static_cast<float>(occupancy_resolution_);
    occupancy_msg_.info.width = occupancy_width_;
    occupancy_msg_.info.height = occupancy_height_;
    occupancy_msg_.info.origin.position.x = occupancy_origin_x_;
    occupancy_msg_.info.origin.position.y = occupancy_origin_y_;
    occupancy_msg_.info.origin.position.z = 0.0;
    occupancy_msg_.info.origin.orientation.w = 1.0;
    occupancy_msg_.data = inflateGrid(occupied);
    occupancy_grid_ready_ = true;

    buildBoundaryCloud();

    RCLCPP_INFO(
      get_logger(),
      "PCD occupancy grid ready. raw=%zu kept=%zu occupied_cells=%zu size=%ux%u res=%.3f z=[%.2f, %.2f] min_points=%d min_z_span=%.2f",
      raw->size(), kept.size(), occupied_cells, occupancy_width_, occupancy_height_,
      occupancy_resolution_, occupancy_min_z_, occupancy_max_z_, min_points,
      occupancy_min_z_span_);

    if (occupancy_save_map_files_) {
      saveOccupancyFiles();
    }
  }

  void buildBoundaryCloud()
  {
    CloudT boundary;
    boundary.header.frame_id = global_frame_;

    for (int y = 1; y < static_cast<int>(occupancy_height_) - 1; ++y) {
      for (int x = 1; x < static_cast<int>(occupancy_width_) - 1; ++x) {
        if (occupancy_msg_.data[gridIndex(x, y)] != 100) {
          continue;
        }
        const bool edge =
          occupancy_msg_.data[gridIndex(x - 1, y)] != 100 ||
          occupancy_msg_.data[gridIndex(x + 1, y)] != 100 ||
          occupancy_msg_.data[gridIndex(x, y - 1)] != 100 ||
          occupancy_msg_.data[gridIndex(x, y + 1)] != 100;
        if (!edge) {
          continue;
        }

        PointT point;
        point.x = static_cast<float>(occupancy_origin_x_ + (x + 0.5) * occupancy_resolution_);
        point.y = static_cast<float>(occupancy_origin_y_ + (y + 0.5) * occupancy_resolution_);
        point.z = 0.05f;
        point.intensity = 255.0f;
        boundary.points.push_back(point);
      }
    }

    boundary.width = static_cast<std::uint32_t>(boundary.points.size());
    boundary.height = 1;
    boundary.is_dense = true;
    pcl::toROSMsg(boundary, boundary_msg_);
    boundary_msg_.header.frame_id = global_frame_;
  }

  void saveOccupancyFiles()
  {
    std::string yaml_path = occupancy_yaml_path_;
    if (yaml_path.empty()) {
      yaml_path = map_path_ + ".grid.yaml";
    }
    const std::string pgm_path = yaml_path + ".pgm";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_WARN(get_logger(), "failed to write occupancy PGM: %s", pgm_path.c_str());
      return;
    }
    pgm << "P5\n" << occupancy_width_ << " " << occupancy_height_ << "\n255\n";
    for (int y = static_cast<int>(occupancy_height_) - 1; y >= 0; --y) {
      for (int x = 0; x < static_cast<int>(occupancy_width_); ++x) {
        const auto value = occupancy_msg_.data[gridIndex(x, y)] == 100 ? 0 : 254;
        const auto byte = static_cast<unsigned char>(value);
        pgm.write(reinterpret_cast<const char *>(&byte), 1);
      }
    }

    std::ofstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_WARN(get_logger(), "failed to write occupancy YAML: %s", yaml_path.c_str());
      return;
    }
    yaml << "image: " << pgm_path << "\n";
    yaml << "mode: trinary\n";
    yaml << "resolution: " << occupancy_resolution_ << "\n";
    yaml << "origin: [" << occupancy_origin_x_ << ", " << occupancy_origin_y_ << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.25\n";

    RCLCPP_INFO(get_logger(), "saved occupancy grid: %s and %s", yaml_path.c_str(), pgm_path.c_str());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_odom_ = odomToMatrix(*msg);
    have_odom_ = true;
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // RViz /initialpose describes the physical chassis (base_link), while
    // NDT estimates the LiDAR/Fast-LIO body frame.  Convert the user input
    // into the frame used by NDT so the 2D Pose Estimate arrow is intuitive.
    Eigen::Matrix4f base_t_localization = localizationToBase().inverse();
    // RViz 2D Pose Estimate always supplies z=0, while this PCD map uses the
    // LiDAR elevation as its vertical reference. Apply the calibrated planar
    // extrinsic here, but do not lift the NDT initial guess by the sensor
    // mounting height.
    base_t_localization(2, 3) = 0.0F;
    const Eigen::Matrix4f initial_pose = poseWithCovToMatrix(*msg) * base_t_localization;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_t_body_ = initial_pose;
      if (have_odom_) {
        last_odom_ = current_odom_;
        have_last_odom_ = true;
      }
      initialized_ = true;
      have_ndt_pose_ = false;
      last_fitness_ = 999.0;
      last_status_ = "INITIAL POSE";
    }
    RCLCPP_WARN(
      get_logger(),
      "Initial pose reset from /initialpose. Waiting for %s and %s before NDT updates.",
      input_cloud_topic_.c_str(), odom_topic_.c_str());
    publishPoseEstimate(now(), initial_pose, last_fitness_, "INITIAL POSE", true);
  }

  bool isPoseInRejectedCell(const Eigen::Matrix4f & pose, std::string & reason) const
  {
    if (!reject_occupied_pose_) {
      return false;
    }
    if (!occupancy_grid_ready_ || occupancy_width_ == 0 || occupancy_height_ == 0) {
      reason = "occupancy grid is not ready";
      return true;
    }

    // NDT estimates the Livox/body origin, while occupancy validity is a
    // physical chassis constraint.  Check the calibrated base_link center so
    // the forward and tilted sensor offset cannot reject a valid start cell.
    const Eigen::Matrix4f base_pose = pose * localizationToBase();
    const auto gx = static_cast<int>(std::floor(
      (base_pose(0, 3) - occupancy_origin_x_) / occupancy_resolution_));
    const auto gy = static_cast<int>(std::floor(
      (base_pose(1, 3) - occupancy_origin_y_) / occupancy_resolution_));
    if (gx < 0 || gy < 0 ||
        gx >= static_cast<int>(occupancy_width_) ||
        gy >= static_cast<int>(occupancy_height_)) {
      reason = "pose is outside occupancy grid";
      return reject_pose_outside_grid_;
    }

    const int radius = std::max(0, occupancy_check_radius_cells_);
    int occupied_count = 0;
    const int reject_count = std::max(1, occupancy_reject_min_occupied_cells_);
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int nx = gx + dx;
        const int ny = gy + dy;
        if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(occupancy_width_) ||
            ny >= static_cast<int>(occupancy_height_)) {
          continue;
        }
        if (occupancy_msg_.data[gridIndex(nx, ny)] == 100) {
          ++occupied_count;
          if (occupied_count >= reject_count) {
            reason = "pose overlaps too many occupied PCD grid cells";
            return true;
          }
        }
      }
    }

    return false;
  }

  Eigen::Matrix4f makeInitialGuess()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      if (have_odom_) {
        last_odom_ = current_odom_;
        have_last_odom_ = true;
      }
      initialized_ = true;
      map_t_body_ = Eigen::Matrix4f::Identity();
      return map_t_body_;
    }

    if (have_odom_ && have_last_odom_) {
      const Eigen::Matrix4f delta = last_odom_.inverse() * current_odom_;
      return map_t_body_ * delta;
    }
    return map_t_body_;
  }

  void acceptPose(const Eigen::Matrix4f & pose, double fitness)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_t_body_ = pose;
    if (have_odom_) {
      last_odom_ = current_odom_;
      have_last_odom_ = true;
    }
    have_ndt_pose_ = true;
    last_fitness_ = fitness;
    last_status_ = "LOCALIZED";
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (require_initial_pose_ && !initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /initialpose before running NDT localization");
      return;
    }

    CloudT::Ptr scan_raw(new CloudT);
    pcl::fromROSMsg(*msg, *scan_raw);
    CloudT::Ptr scan = voxelDownsample(scan_raw, scan_leaf_size_);
    if (static_cast<int>(scan->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skip scan: too few points after filtering (%zu)", scan->size());
      return;
    }

    const Eigen::Matrix4f guess = makeInitialGuess();
    CloudT aligned;
    ndt_.setInputSource(scan);
    ndt_.align(aligned, guess);

    const double fitness = ndt_.getFitnessScore();
    const bool ok = ndt_.hasConverged() && fitness <= max_fitness_score_;
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "NDT rejected: converged=%d fitness=%.4f points=%zu",
        ndt_.hasConverged(), fitness, scan->size());
      Eigen::Matrix4f held_pose;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        held_pose = map_t_body_;
        last_fitness_ = fitness;
        have_ndt_pose_ = false;
        last_status_ = "NDT REJECTED";
      }
      // Keep the last accepted pose, but publish the failed score immediately
      // so the motion safety gate cannot mistake an old good score for a
      // currently healthy localization.
      publishPoseEstimate(
        msg->header.stamp, held_pose, fitness, "NDT REJECTED", false);
      return;
    }

    const Eigen::Matrix4f pose = ndt_.getFinalTransformation();
    std::string reject_reason;
    if (isPoseInRejectedCell(pose, reject_reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NDT rejected: %s pose=(%.2f %.2f %.2f) fitness=%.4f",
        reject_reason.c_str(), pose(0, 3), pose(1, 3), pose(2, 3), fitness);
      Eigen::Matrix4f held_pose;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        held_pose = map_t_body_;
        last_fitness_ = std::max(fitness, max_fitness_score_ + 1.0e-6);
        have_ndt_pose_ = false;
        last_status_ = "NDT REJECTED: OCCUPIED";
      }
      publishPoseEstimate(
        msg->header.stamp, held_pose, last_fitness_, last_status_, false);
      return;
    }

    acceptPose(pose, fitness);
    publishResult(*msg, pose, aligned, fitness);
  }

  void publishResult(
    const sensor_msgs::msg::PointCloud2 & input_msg,
    const Eigen::Matrix4f & pose,
    const CloudT & aligned,
    double fitness)
  {
    const auto stamp = input_msg.header.stamp;
    const auto stamped = publishPoseEstimate(stamp, pose, fitness, "LOCALIZED", true);

    if (publish_aligned_cloud_) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(aligned, out);
      out.header.stamp = stamp;
      out.header.frame_id = global_frame_;
      aligned_pub_->publish(out);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "NDT ok fitness=%.4f pose=(%.2f %.2f %.2f)",
      fitness, pose(0, 3), pose(1, 3), pose(2, 3));
  }

  geometry_msgs::msg::PoseStamped publishPoseEstimate(
    const rclcpp::Time & stamp,
    const Eigen::Matrix4f & pose,
    double fitness,
    const std::string & status,
    bool publish_path)
  {
    geometry_msgs::msg::Pose pose_msg = matrixToPose(pose);

    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.stamp = stamp;
    stamped.header.frame_id = global_frame_;
    stamped.pose = pose_msg;
    pose_pub_->publish(stamped);

    // Keep NDT and /relocalization_pose in the LiDAR localization frame, but
    // draw the marker/path at the physical chassis center. This makes the
    // RViz heading and position agree with base_link while preserving the map
    // matching frame used by existing consumers.
    geometry_msgs::msg::PoseStamped base_stamped = stamped;
    base_stamped.pose = matrixToPose(pose * localizationToBase());

    nav_msgs::msg::Odometry odom;
    odom.header = stamped.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose_msg;
    odom.pose.covariance[0] = fitness;
    odom.pose.covariance[7] = fitness;
    odom.pose.covariance[14] = fitness;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = stamped.header;
      tf.header.stamp = now();
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose(0, 3);
      tf.transform.translation.y = pose(1, 3);
      tf.transform.translation.z = pose(2, 3);
      tf.transform.rotation = pose_msg.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    publishMarkers(base_stamped, fitness, status);
    if (publish_path) {
      publishPath(base_stamped);
    }
    return stamped;
  }

  void publishMap()
  {
    const auto stamp = now();
    if (publish_map_cloud_) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*map_cloud_, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = global_frame_;
      map_pub_->publish(msg);
    }

    if (publish_occupancy_grid_) {
      occupancy_msg_.header.stamp = stamp;
      occupancy_pub_->publish(occupancy_msg_);

      boundary_msg_.header.stamp = stamp;
      boundary_pub_->publish(boundary_msg_);
    }

    Eigen::Matrix4f pose;
    bool initialized = false;
    bool have_ndt_pose = false;
    double fitness = 999.0;
    std::string status = "INITIAL POSE";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      initialized = initialized_;
      have_ndt_pose = have_ndt_pose_;
      fitness = last_fitness_;
      status = last_status_;
      pose = map_t_body_;
    }
    if (initialized) {
      publishPoseEstimate(
        stamp, pose, fitness, have_ndt_pose ? "LOCALIZED" : status, false);
    }
  }

  void publishMarkers(
    const geometry_msgs::msg::PoseStamped & pose,
    double fitness,
    const std::string & status)
  {
    visualization_msgs::msg::MarkerArray markers;
    const bool localized = status == "LOCALIZED";
    const bool rejected = status.rfind("NDT REJECTED", 0) == 0;
    const auto marker_lifetime = rclcpp::Duration::from_seconds(
      std::max(0.0, marker_lifetime_sec_));

    visualization_msgs::msg::Marker body;
    body.header = pose.header;
    body.ns = "relocalization";
    body.id = 1;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose = pose.pose;
    body.scale.x = 0.45;
    body.scale.y = 0.45;
    body.scale.z = 0.45;
    body.color.r = rejected ? 1.0f : (localized ? 0.0f : 1.0f);
    body.color.g = rejected ? 0.1f : (localized ? 1.0f : 0.55f);
    body.color.b = localized ? 1.0f : 0.0f;
    body.color.a = 1.0f;
    body.lifetime = marker_lifetime;
    markers.markers.push_back(body);

    visualization_msgs::msg::Marker heading;
    heading.header = pose.header;
    heading.ns = "relocalization";
    heading.id = 2;
    heading.type = visualization_msgs::msg::Marker::ARROW;
    heading.action = visualization_msgs::msg::Marker::ADD;
    heading.pose = pose.pose;
    const Eigen::AngleAxisf visual_yaw(
      static_cast<float>(visualization_yaw_offset_rad_), Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf heading_q(
      static_cast<float>(heading.pose.orientation.w),
      static_cast<float>(heading.pose.orientation.x),
      static_cast<float>(heading.pose.orientation.y),
      static_cast<float>(heading.pose.orientation.z));
    heading_q = heading_q * Eigen::Quaternionf(visual_yaw);
    heading_q.normalize();
    heading.pose.orientation.x = heading_q.x();
    heading.pose.orientation.y = heading_q.y();
    heading.pose.orientation.z = heading_q.z();
    heading.pose.orientation.w = heading_q.w();
    heading.scale.x = 1.0;
    heading.scale.y = 0.16;
    heading.scale.z = 0.16;
    heading.color.r = 1.0f;
    heading.color.g = rejected ? 0.1f : 1.0f;
    heading.color.b = 0.0f;
    heading.color.a = 1.0f;
    heading.lifetime = marker_lifetime;
    markers.markers.push_back(heading);

    visualization_msgs::msg::Marker text;
    text.header = pose.header;
    text.ns = "relocalization";
    text.id = 3;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = pose.pose;
    text.pose.position.z += 0.8;
    text.scale.z = 0.35;
    text.color.r = rejected ? 1.0f : 1.0f;
    text.color.g = rejected ? 0.2f : 1.0f;
    text.color.b = rejected ? 0.2f : 1.0f;
    text.color.a = 1.0f;
    text.text = status + "  score=" + std::to_string(fitness).substr(0, 5);
    text.lifetime = marker_lifetime;
    markers.markers.push_back(text);

    marker_pub_->publish(markers);
  }

  void publishPath(const geometry_msgs::msg::PoseStamped & pose)
  {
    path_msg_.header = pose.header;
    path_msg_.poses.push_back(pose);

    if (path_keep_count_ > 0 &&
        path_msg_.poses.size() > static_cast<std::size_t>(path_keep_count_)) {
      const auto remove_count =
        path_msg_.poses.size() - static_cast<std::size_t>(path_keep_count_);
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + remove_count);
    }

    path_pub_->publish(path_msg_);
  }

  std::mutex mutex_;
  std::string map_path_;
  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string initialpose_topic_;
  std::string global_frame_;
  std::string base_frame_;
  double localization_to_base_roll_rad_;
  double localization_to_base_pitch_rad_;
  double localization_to_base_yaw_rad_;
  double localization_to_base_x_m_;
  double localization_to_base_y_m_;
  double localization_to_base_z_m_;
  double visualization_yaw_offset_rad_;
  double marker_lifetime_sec_;
  double map_leaf_size_;
  double scan_leaf_size_;
  int min_scan_points_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_trans_eps_;
  int ndt_max_iterations_;
  double max_fitness_score_;
  int path_keep_count_;
  bool require_initial_pose_;
  bool publish_tf_;
  bool publish_map_cloud_;
  bool publish_aligned_cloud_;
  double map_publish_period_sec_;
  bool publish_occupancy_grid_;
  double occupancy_resolution_;
  double occupancy_min_z_;
  double occupancy_max_z_;
  double occupancy_padding_m_;
  int occupancy_inflate_cells_;
  int occupancy_min_points_per_cell_;
  double occupancy_min_z_span_;
  bool occupancy_save_map_files_;
  std::string occupancy_yaml_path_;
  bool reject_occupied_pose_;
  bool reject_pose_outside_grid_;
  int occupancy_check_radius_cells_;
  int occupancy_reject_min_occupied_cells_;
  double occupancy_origin_x_;
  double occupancy_origin_y_;
  std::uint32_t occupancy_width_;
  std::uint32_t occupancy_height_;
  bool occupancy_grid_ready_ = false;

  CloudT::Ptr map_cloud_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  Eigen::Matrix4f map_t_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_odom_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f last_odom_ = Eigen::Matrix4f::Identity();
  bool initialized_ = false;
  bool have_ndt_pose_ = false;
  bool have_odom_ = false;
  bool have_last_odom_ = false;
  double last_fitness_ = 999.0;
  std::string last_status_ = "INITIAL POSE";

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  nav_msgs::msg::Path path_msg_;
  nav_msgs::msg::OccupancyGrid occupancy_msg_;
  sensor_msgs::msg::PointCloud2 boundary_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PcdNdtLocalizer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_ndt_localizer"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
