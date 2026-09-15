import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the parameter file
    bringup_dir = get_package_share_directory('pipe_track_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'params.yaml')
    holo_config_file = os.path.join(bringup_dir, 'config', 'holoocean_config.json')

    return LaunchDescription([
        Node(
            package='pipe_track_sim',
            executable='sim_node',
            name='sim_node',
            parameters=[config_file, {'holoocean_config_path': holo_config_file}]
        ),
        Node(
            package='pipe_track_control',
            executable='waypoint_control_node',
            name='waypoint_control_node',
            parameters=[config_file]
        ),
        Node(
            package='pipe_track_perception',
            executable='segmentation',
            name='segmentation_node',
            parameters=[config_file]
        ),
        Node(
            package='pipe_track_planning',
            executable='point_extract',
            name='point_extract',
            parameters=[config_file]
        ),
        Node(
            package='pipe_track_mission',
            executable='behaviour_node',
            name='behaviour_node',
            parameters=[config_file]
        ),
        Node(
            package='pipe_track_evaluation',
            executable='evaluation_system',
            name='evaluation_system',
            parameters=[config_file]
        )
    ])