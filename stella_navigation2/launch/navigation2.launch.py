import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Set Packages Directory
    bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'))
    stella_desc_dir = get_package_share_directory('stella_description')
    stella_nav2_dir = get_package_share_directory('stella_navigation2')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # LaunchConfiguration Values (For Real Robot)
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false', description='Whether to apply a namespace to the navigation stack'
    )
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether to run SLAM'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stella_nav2_dir, 'maps', 'stella_world.yaml'),
        description='Full path to map file to load'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stella_nav2_dir, 'param', 'stella.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically start the nav2 stack'
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Whether to use composed bringup'
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='Whether to respawn if a node crashes'
    )   
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(stella_nav2_dir, 'rviz', 'stella_navigation2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    params_file_path = os.path.join(stella_nav2_dir, 'param', 'stella_senser.yaml')
    with open(params_file_path, 'r') as f:
        params = yaml.safe_load(f)

    camera = params.get('camera', 'default')

    if camera == 'use_realsense':
        urdf_file = os.path.join(stella_desc_dir, 'urdf', 'stella_realsense.urdf')
    elif camera == 'use_web_cam':
        urdf_file = os.path.join(stella_desc_dir, 'urdf', 'stella_web_cam.urdf')
    else:
        # Default
        urdf_file = os.path.join(stella_desc_dir, 'urdf', 'stella.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )
    
    log_map = LogInfo(msg=['Using map file: ', map_yaml_file])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(log_map)

    ld.add_action(bringup_cmd)
    
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(rviz_cmd)
    
    return ld

