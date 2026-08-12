from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("pcd_ndt_localization"),
        "config",
        "ndt_localization.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "map_path",
            default_value="/home/seeed/ros2/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729.pcd",
        ),
        DeclareLaunchArgument("publish_grid_map", default_value="true"),
        DeclareLaunchArgument(
            "initialpose_topic", default_value="/initialpose_relay"),
        DeclareLaunchArgument("grid_resolution", default_value="0.10"),
        DeclareLaunchArgument("grid_min_z", default_value="-0.635880326"),
        DeclareLaunchArgument("grid_max_z", default_value="0.844119674"),
        DeclareLaunchArgument("grid_inflate_cells", default_value="0"),
        DeclareLaunchArgument("grid_min_points_per_cell", default_value="3"),
        DeclareLaunchArgument("grid_min_z_span", default_value="0.10"),
        DeclareLaunchArgument("save_grid_map", default_value="false"),
        DeclareLaunchArgument(
            "grid_yaml_path",
            default_value="/home/seeed/ros2/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml",
        ),
        Node(
            package="pcd_ndt_localization",
            executable="pcd_ndt_localizer",
            name="pcd_ndt_localizer",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "map_path": LaunchConfiguration("map_path"),
                    "initialpose_topic": LaunchConfiguration(
                        "initialpose_topic"),
                    "publish_occupancy_grid": ParameterValue(
                        LaunchConfiguration("publish_grid_map"),
                        value_type=bool,
                    ),
                    "occupancy_resolution": ParameterValue(
                        LaunchConfiguration("grid_resolution"),
                        value_type=float,
                    ),
                    "occupancy_min_z": ParameterValue(
                        LaunchConfiguration("grid_min_z"),
                        value_type=float,
                    ),
                    "occupancy_max_z": ParameterValue(
                        LaunchConfiguration("grid_max_z"),
                        value_type=float,
                    ),
                    "occupancy_inflate_cells": ParameterValue(
                        LaunchConfiguration("grid_inflate_cells"),
                        value_type=int,
                    ),
                    "occupancy_min_points_per_cell": ParameterValue(
                        LaunchConfiguration("grid_min_points_per_cell"),
                        value_type=int,
                    ),
                    "occupancy_min_z_span": ParameterValue(
                        LaunchConfiguration("grid_min_z_span"),
                        value_type=float,
                    ),
                    "occupancy_save_map_files": ParameterValue(
                        LaunchConfiguration("save_grid_map"),
                        value_type=bool,
                    ),
                    "occupancy_yaml_path": LaunchConfiguration("grid_yaml_path"),
                },
            ],
        ),
    ])
