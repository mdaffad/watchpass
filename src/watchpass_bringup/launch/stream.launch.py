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
# Bring up the Watchpass streaming pipeline in a single component container.
#
# Running camera_bridge and ffmpeg_streamer as composable nodes in one process
# with intra-process comms means the WatchFrame handed from the bridge to the
# streamer is never copied or serialized. Even split across processes, the
# Fast DDS shared-memory profile loaded here keeps the transfer zero-copy.

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

    source = LaunchConfiguration('source')
    image_topic = LaunchConfiguration('image_topic')
    device = LaunchConfiguration('device')
    # Numeric params arrive as strings from the CLI; coerce so declare_parameter
    # sees the type the node expects.
    width = ParameterValue(LaunchConfiguration('width'), value_type=int)
    height = ParameterValue(LaunchConfiguration('height'), value_type=int)
    pixel_format = LaunchConfiguration('pixel_format')
    frame_topic = LaunchConfiguration('frame_topic')
    fps = ParameterValue(LaunchConfiguration('fps'), value_type=float)
    rtsp_url = LaunchConfiguration('rtsp_url')
    encoder = LaunchConfiguration('encoder')
    vaapi_device = LaunchConfiguration('vaapi_device')

    args = [
        DeclareLaunchArgument(
            'source', default_value='topic',
            description='Frame source: topic (subscribe) or v4l2 (direct device)'),
        DeclareLaunchArgument(
            'image_topic', default_value='image_raw',
            description='sensor_msgs/Image topic when source:=topic'),
        DeclareLaunchArgument(
            'device', default_value='/dev/video0',
            description='V4L2 device when source:=v4l2'),
        DeclareLaunchArgument(
            'width', default_value='640',
            description='Requested capture width when source:=v4l2'),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='Requested capture height when source:=v4l2'),
        DeclareLaunchArgument(
            'pixel_format', default_value='rgb8',
            description='V4L2 capture format: rgb8, bgr8 or mono8'),
        DeclareLaunchArgument(
            'frame_topic', default_value='watch_frame',
            description='WatchFrame topic used between the bridge and streamer'),
        DeclareLaunchArgument(
            'fps', default_value='30.0',
            description='Frame rate (advertised to ffmpeg; requested from V4L2)'),
        DeclareLaunchArgument(
            'rtsp_url', default_value='rtsp://127.0.0.1:8554/watchpass',
            description='Destination the encoded stream is pushed to'),
        DeclareLaunchArgument(
            'encoder', default_value='x264',
            description='Encoder preset: x264 (software) or vaapi (AMD/Intel GPU)'),
        DeclareLaunchArgument(
            'vaapi_device', default_value='/dev/dri/renderD128',
            description='VA-API render node used when encoder:=vaapi'),
    ]

    # Force the shared-memory / data-sharing path.
    set_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_profile)

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
                parameters=[{
                    'source': source,
                    'input_topic': image_topic,
                    'device': device,
                    'width': width,
                    'height': height,
                    'pixel_format': pixel_format,
                    'fps': fps,
                    'output_topic': frame_topic,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='watchpass_streamer',
                plugin='watchpass_streamer::FfmpegStreamerNode',
                name='ffmpeg_streamer',
                parameters=[{
                    'input_topic': frame_topic,
                    'fps': fps,
                    'encoder': encoder,
                    'rtsp_url': rtsp_url,
                    'vaapi_device': vaapi_device,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [set_profile, container])
