from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Status subscriber
    status_subscriber = Node(
            package='medi_buddy_server',
            executable='status_subscriber',
            output='screen'
        )

    # Voice router
    voice_router = Node(
            package='medi_buddy_server',
            executable='voice_router',
            output='screen'
        )

    # OCR publisher
    ocr_publisher = Node(
            package='medi_buddy_server',
            executable='ocr_publisher',
            output='screen'
        )

    return LaunchDescription([
        status_subscriber,
        voice_router,
        ocr_publisher,
    ])

