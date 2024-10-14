#! /usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   camera_align_depth = LaunchConfiguration('camera_align_depth')
   camera_pointcloud = LaunchConfiguration('camera_pointcloud')
   camera_depth = LaunchConfiguration('camera_depth')
   usb_port = LaunchConfiguration('usb_port')

   usb_port_launch_arg = DeclareLaunchArgument(                          
      'usb_port',
      default_value = '/dev/ttyACM0'
    )

   camera_align_depth_launch_arg = DeclareLaunchArgument(
      'camera_align_depth',
      default_value = 'False'
    )

   camera_pointcloud_launch_arg = DeclareLaunchArgument(
      'camera_pointcloud',
      default_value = 'False'
    )

   camera_depth_launch_arg = DeclareLaunchArgument(
      'camera_depth',
      default_value = 'True'
    )

   camera_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'), 'launch'),
         '/rs_launch.py']),
         
      launch_arguments = {
         'align_depth.enable': camera_align_depth,
         'pointcloud.enable': camera_pointcloud,
         'enable_depth': camera_depth
      }.items(),
    )

   lidar_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('sllidar_ros2'), 'launch'),
         '/view_sllidar_s2_launch.py'])
    )
   
   return LaunchDescription(
      usb_port_launch_arg,
      camera_align_depth_launch_arg,
      camera_pointcloud_launch_arg,
      camera_depth_launch_arg,
      camera_node,
      lidar_node
   )
