#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

bool finitePoint(const PointT & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}
}  // namespace

class PcdOccupancyGridPublisher : public rclcpp::Node
{
public:
  PcdOccupancyGridPublisher() : Node("pcd_occupancy_grid_publisher")
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    min_z_ = declare_parameter<double>("min_z", -0.2);
    max_z_ = declare_parameter<double>("max_z", 2.0);
    padding_m_ = declare_parameter<double>("padding_m", 1.0);
    inflate_cells_ = declare_parameter<int>("inflate_cells", 1);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 3);
    min_z_span_ = declare_parameter<double>("min_z_span", 0.15);
    publish_period_sec_ = declare_parameter<double>("publish_period_sec", 2.0);
    save_map_files_ = declare_parameter<bool>("save_map_files", true);
    map_yaml_path_ = declare_parameter<std::string>("map_yaml_path", "");

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/pcd_occupancy_grid", rclcpp::QoS(1).transient_local().reliable());
    boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_boundary_cloud", rclcpp::QoS(1).transient_local().reliable());

    buildMaps();

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_sec_)),
      std::bind(&PcdOccupancyGridPublisher::publishMaps, this));
  }

private:
  void buildMaps()
  {
    if (map_path_.empty()) {
      throw std::runtime_error("map_path is empty");
    }
    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be positive");
    }
    if (min_z_ >= max_z_) {
      throw std::runtime_error("min_z must be smaller than max_z");
    }

    CloudT raw;
    if (pcl::io::loadPCDFile<PointT>(map_path_, raw) != 0) {
      throw std::runtime_error("failed to load PCD map: " + map_path_);
    }

    std::vector<PointT> kept;
    kept.reserve(raw.size());
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto & point : raw.points) {
      if (!finitePoint(point) || point.z < min_z_ || point.z > max_z_) {
        continue;
      }
      kept.push_back(point);
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    if (kept.empty()) {
      throw std::runtime_error("no PCD points kept after z filter; adjust min_z/max_z");
    }

    origin_x_ = min_x - padding_m_;
    origin_y_ = min_y - padding_m_;
    const double span_x = max_x - min_x + 2.0 * padding_m_;
    const double span_y = max_y - min_y + 2.0 * padding_m_;
    width_ = static_cast<std::uint32_t>(std::ceil(span_x / resolution_));
    height_ = static_cast<std::uint32_t>(std::ceil(span_y / resolution_));

    struct CellStats
    {
      int count = 0;
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
    };

    std::vector<CellStats> stats(width_ * height_);
    for (const auto & point : kept) {
      const int x = static_cast<int>(std::floor((point.x - origin_x_) / resolution_));
      const int y = static_cast<int>(std::floor((point.y - origin_y_) / resolution_));
      if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_)) {
        continue;
      }
      auto & cell = stats[index(x, y)];
      ++cell.count;
      cell.min_z = std::min(cell.min_z, point.z);
      cell.max_z = std::max(cell.max_z, point.z);
    }

    std::vector<std::int8_t> occupied(width_ * height_, 0);
    const int min_points = std::max(1, min_points_per_cell_);
    std::size_t occupied_cells = 0;
    for (std::size_t i = 0; i < stats.size(); ++i) {
      const auto & cell = stats[i];
      if (cell.count < min_points) {
        continue;
      }
      const float z_span = cell.max_z - cell.min_z;
      if (min_z_span_ > 0.0 && z_span < static_cast<float>(min_z_span_)) {
        continue;
      }
      occupied[i] = 100;
      ++occupied_cells;
    }

    grid_msg_.header.frame_id = global_frame_;
    grid_msg_.info.resolution = static_cast<float>(resolution_);
    grid_msg_.info.width = width_;
    grid_msg_.info.height = height_;
    grid_msg_.info.origin.position.x = origin_x_;
    grid_msg_.info.origin.position.y = origin_y_;
    grid_msg_.info.origin.position.z = 0.0;
    grid_msg_.info.origin.orientation.w = 1.0;
    grid_msg_.data = inflate(occupied);

    buildBoundaryCloud();

    RCLCPP_INFO(
      get_logger(),
      "PCD occupancy grid ready. raw=%zu kept=%zu occupied_cells=%zu size=%ux%u res=%.3f z=[%.2f, %.2f] min_points=%d min_z_span=%.2f",
      raw.size(), kept.size(), occupied_cells, width_, height_, resolution_, min_z_, max_z_,
      min_points, min_z_span_);

    if (save_map_files_) {
      saveMapFiles();
    }
  }

  std::size_t index(int x, int y) const
  {
    return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
  }

  std::vector<std::int8_t> inflate(const std::vector<std::int8_t> & input) const
  {
    if (inflate_cells_ <= 0) {
      return input;
    }

    std::vector<std::int8_t> output = input;
    for (int y = 0; y < static_cast<int>(height_); ++y) {
      for (int x = 0; x < static_cast<int>(width_); ++x) {
        if (input[index(x, y)] != 100) {
          continue;
        }
        for (int dy = -inflate_cells_; dy <= inflate_cells_; ++dy) {
          for (int dx = -inflate_cells_; dx <= inflate_cells_; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
              output[index(nx, ny)] = 100;
            }
          }
        }
      }
    }
    return output;
  }

  void buildBoundaryCloud()
  {
    CloudT boundary;
    boundary.header.frame_id = global_frame_;

    for (int y = 1; y < static_cast<int>(height_) - 1; ++y) {
      for (int x = 1; x < static_cast<int>(width_) - 1; ++x) {
        if (grid_msg_.data[index(x, y)] != 100) {
          continue;
        }
        const bool edge =
          grid_msg_.data[index(x - 1, y)] != 100 ||
          grid_msg_.data[index(x + 1, y)] != 100 ||
          grid_msg_.data[index(x, y - 1)] != 100 ||
          grid_msg_.data[index(x, y + 1)] != 100;
        if (!edge) {
          continue;
        }

        PointT point;
        point.x = static_cast<float>(origin_x_ + (x + 0.5) * resolution_);
        point.y = static_cast<float>(origin_y_ + (y + 0.5) * resolution_);
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

  void saveMapFiles()
  {
    std::string yaml_path = map_yaml_path_;
    if (yaml_path.empty()) {
      yaml_path = map_path_ + ".grid.yaml";
    }
    const std::string pgm_path = yaml_path + ".pgm";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_WARN(get_logger(), "failed to write occupancy PGM: %s", pgm_path.c_str());
      return;
    }
    pgm << "P5\n" << width_ << " " << height_ << "\n255\n";
    for (int y = static_cast<int>(height_) - 1; y >= 0; --y) {
      for (int x = 0; x < static_cast<int>(width_); ++x) {
        const auto value = grid_msg_.data[index(x, y)] == 100 ? 0 : 254;
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
    yaml << "resolution: " << resolution_ << "\n";
    yaml << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.25\n";

    RCLCPP_INFO(get_logger(), "saved occupancy grid: %s and %s", yaml_path.c_str(), pgm_path.c_str());
  }

  void publishMaps()
  {
    const auto stamp = now();
    grid_msg_.header.stamp = stamp;
    grid_pub_->publish(grid_msg_);

    boundary_msg_.header.stamp = stamp;
    boundary_pub_->publish(boundary_msg_);
  }

  std::string map_path_;
  std::string global_frame_;
  double resolution_;
  double min_z_;
  double max_z_;
  double padding_m_;
  int inflate_cells_;
  int min_points_per_cell_;
  double min_z_span_;
  double publish_period_sec_;
  bool save_map_files_;
  std::string map_yaml_path_;

  double origin_x_;
  double origin_y_;
  std::uint32_t width_;
  std::uint32_t height_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
  sensor_msgs::msg::PointCloud2 boundary_msg_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PcdOccupancyGridPublisher>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_occupancy_grid_publisher"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
