#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # Launch arguments:
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Find package share directory:
    pkg_gz = FindPackageShare("bastion_gazebo")

    class Model:
        def __init__(self, model_name, gz_name, coordinates):
            self.model_name = model_name
            self.gz_name = gz_name
            self.coordinates = coordinates
            self.path = PathJoinSubstitution([
                pkg_gz,
                "models",
                self.model_name,
                "sdf",
                self.model_name + ".sdf"
            ])
            self.spawn_node = Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_' + self.model_name,
                output='screen',
                arguments=[
                    '-entity', self.gz_name,
                    '-file', self.path,
                    '-allow_renaming', 'true',
                    '-x', str(self.coordinates[0]),
                    '-y', str(self.coordinates[1]),
                    '-z', str(self.coordinates[2]),
                    '-R', str(self.coordinates[3]),
                    '-P', str(self.coordinates[4]),
                    '-Y', str(self.coordinates[5]),
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
    
    desk = Model("desk", "office_desk", [-0.19, -0.03, 0.54, 0.0, 0.0, 0.0])
    lamp = Model("lamp", "desk_lamp", [-0.68, 0.20, 0.84, 0.0, 0.0, 2.08])

    return LaunchDescription([
        use_sim_time_arg,
        desk.spawn_node,
        lamp.spawn_node,
    ])