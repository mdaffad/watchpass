# Copyright 2026 Muhammad Daffa Dinaya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# End-to-end demo: Gazebo camera -> ros_gz_bridge -> Watchpass pipeline.
#
# Starts gz-sim with a self-contained camera world, bridges the camera image to
# a ROS sensor_msgs/Image topic, and includes stream.launch.py to bridge it into
# zero-copy WatchFrame messages and stream it out through ffmpeg.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('watchpass_bringup')
    world = LaunchConfiguration('world')
    rtsp_url = LaunchConfiguration('rtsp_url')

    args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_share, 'worlds', 'watchpass_camera.sdf'),
            description='SDF world to load in Gazebo'),
        DeclareLaunchArgument(
            'rtsp_url', default_value='rtsp://127.0.0.1:8554/watchpass',
            description='Destination the encoded stream is pushed to'),
    ]

    # Gazebo (gz-sim). ros_gz_sim ships this launch file.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v3 ', world]}.items(),
    )

    # Bridge the gz camera image topic to ROS as sensor_msgs/Image on image_raw.
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[('/camera', 'image_raw')],
        output='screen',
    )

    stream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'stream.launch.py')),
        launch_arguments={
            'image_topic': 'image_raw',
            'rtsp_url': rtsp_url,
        }.items(),
    )

    return LaunchDescription(args + [gz_sim, camera_bridge, stream])
