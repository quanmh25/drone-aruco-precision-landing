import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_sim = get_package_share_directory('landing_description')
    pkg_node = get_package_share_directory('landing_control')


    # Launch Simulation (Gazebo, RViz, Spawn robot, Bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim.launch.py')
        )
    )

    ground_truth_file = os.path.join(
        get_package_share_directory('landing_control'), 'config', 'ground_truth.yaml')

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_node',
        arguments=['/camera/image_debug'], 
        output='screen'
    )

    mission_node = Node(
        package='landing_control',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'target_id': 3,
            'ground_truth_yaml': ground_truth_file,
        }]
    )

    return LaunchDescription([
        sim_launch,
        # TimerAction(period=7.0, actions=[coverage_node]),
        TimerAction(period=7.0, actions=[mission_node]),
        # rqt_image_view_node,
    ])
