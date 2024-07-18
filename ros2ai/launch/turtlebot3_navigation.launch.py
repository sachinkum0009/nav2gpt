#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlebot3_house = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtlebot3_gazebo'), 'launch'),
         '/turtlebot3_house.launch.py'])
      )
#    turtlesim_world_2 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_world_2_launch.py'])
#       )
#    rviz_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_rviz_launch.py'])
#       )

   return LaunchDescription([
      turtlebot3_house
   ])