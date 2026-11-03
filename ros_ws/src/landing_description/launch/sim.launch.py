import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_drone_arena = get_package_share_directory('landing_description')

    models_path = os.path.join(pkg_drone_arena, 'models')
    workspace_share = os.path.dirname(pkg_drone_arena)
    gz_resource_path = models_path + ':' + workspace_share

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # 2. Khai báo các đường dẫn
    world_path = PathJoinSubstitution([FindPackageShare('landing_description'), 'worlds', 'arena.sdf'])
    default_xacro_file = os.path.join(pkg_drone_arena, 'urdf', 'drone.xacro')
    rviz_config_file = os.path.join(pkg_drone_arena, 'config', 'model.rviz') 

    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=default_xacro_file,
        description='Full path to the Xacro file'
    )

    # 4. Khởi chạy Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 5. Robot State Publisher (Bổ sung use_sim_time)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}
        ]
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'parrot_bebop',
            '-x', '1', '-y', '0.34', '-z', '0.1'
            # '-x', '4', '-y', '3', '-z', '0.1',
        ],
        output='screen'
    )


    bridge_config_file = os.path.join(pkg_drone_arena, 'config', 'ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )


    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        set_env,
        declare_urdf_model_cmd,
        gz_sim,
        robot_state_publisher,
        spawn_drone,
        bridge,
        rviz2
    ])
