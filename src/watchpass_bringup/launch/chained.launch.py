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
# Chained zero-copy processing demo:
#
#   image_raw --> camera_bridge --> grayscale --> flip --> ffmpeg_streamer --> RTSP
#                 (WatchFrame)     (mono8)      (mirror)  (encode)
#
# All four nodes run as composable nodes in ONE container with intra-process
# comms, so every WatchFrame hop between them is a pointer move -- no copy, no
# serialization. Swap in your own FrameProcessorNode subclasses to extend the
# chain. This is the "framework" in action: ffmpeg is just the final consumer.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('watchpass_bringup')
    fastdds_profile = os.path.join(bringup_share, 'config', 'fastdds_shm.xml')

    image_topic = LaunchConfiguration('image_topic')
    rtsp_url = LaunchConfiguration('rtsp_url')
    encoder = LaunchConfiguration('encoder')
    flip_code = ParameterValue(LaunchConfiguration('flip_code'), value_type=int)

    args = [
        DeclareLaunchArgument(
            'image_topic', default_value='image_raw',
            description='sensor_msgs/Image topic coming from the camera or sim'),
        DeclareLaunchArgument(
            'rtsp_url', default_value='rtsp://127.0.0.1:8554/watchpass',
            description='Destination the encoded stream is pushed to'),
        DeclareLaunchArgument(
            'encoder', default_value='x264',
            description='Encoder preset: x264 (software) or vaapi (AMD/Intel GPU)'),
        DeclareLaunchArgument(
            'flip_code', default_value='1',
            description='cv::flip code: 0=vertical, 1=horizontal, -1=both'),
    ]

    set_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_profile)

    def intra(**params):
        return {
            'parameters': [params],
            'extra_arguments': [{'use_intra_process_comms': True}],
        }

    container = ComposableNodeContainer(
        name='watchpass_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='watchpass_streamer',
                plugin='watchpass_streamer::CameraBridgeNode',
                name='camera_bridge',
                **intra(input_topic=image_topic, output_topic='watch_frame'),
            ),
            ComposableNode(
                package='watchpass_nodes',
                plugin='watchpass_nodes::GrayscaleNode',
                name='grayscale',
                **intra(input_topic='watch_frame', output_topic='watch_frame_gray'),
            ),
            ComposableNode(
                package='watchpass_nodes',
                plugin='watchpass_nodes::FlipNode',
                name='flip',
                **intra(
                    input_topic='watch_frame_gray',
                    output_topic='watch_frame_flipped',
                    flip_code=flip_code),
            ),
            ComposableNode(
                package='watchpass_streamer',
                plugin='watchpass_streamer::FfmpegStreamerNode',
                name='ffmpeg_streamer',
                **intra(
                    input_topic='watch_frame_flipped',
                    encoder=encoder,
                    rtsp_url=rtsp_url),
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [set_profile, container])
