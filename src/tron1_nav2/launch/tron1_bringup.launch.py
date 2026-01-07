import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directories
    pkg_tron1_nav2 = get_package_share_directory('tron1_nav2')
    pkg_tron1_ctrl = get_package_share_directory('tron1_ctrl_ros2')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robot_description = get_package_share_directory('robot_description') # Assuming this is the package name

    # Configuration Files
    nav2_params_file = os.path.join(pkg_tron1_nav2, 'config', 'nav2_params.yaml')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 1. Robot Description (URDF)
    # We need to find where robot.xacro is launched. 
    # Usually robot_description package has a launch file or we run robot_state_publisher directly.
    # The user provided path: src/robot-description/pointfoot/WF_TRON1B/xacro/robot.xacro
    # We will assume there is a launch file or we construct the command.
    # Since I don't see a clear "bringup" for robot_description in the file list (only xacro), 
    # I will use xacro command to generate URDF and pass to robot_state_publisher.
    
    xacro_file = os.path.join(
        get_package_share_directory('robot_description'),
        'pointfoot', 'WF_TRON1B', 'xacro', 'robot.xacro'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PathJoinSubstitution(['xacro ', xacro_file]) # Command substitution is tricky in Python params
        }]
        # Note: 'robot_description' parameter usually needs the *content* of the URDF. 
        # In ROS2 launch, we typically use Command.
    )
    
    # Correct way to use Command for robot_description
    from launch.substitutions import Command
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # 2. Tron1 Base Control
    tron1_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tron1_ctrl, 'launch', 'tron1.launch.py')
        ),
        # Assuming tron1.launch.py handles necessary args or defaults
    )

    # 3. Fast-LIO SLAM (Mapping Mode)
    # We use mapping.launch.py as identified
    fast_lio_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # We can pass custom config if needed, but default might be sufficient for now.
            # 'config_file': 'mid360.yaml' 
        }.items()
    )

    # 4. Nav2 Navigation
    # We use navigation_launch.py to launch planner, controller, bt_navigator, recoveries
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    # RViz (Optional, Fast-LIO launches RViz by default, Nav2 might want one too)
    # Fast-LIO's RViz config is good for SLAM. We might want to add Nav2 displays to it manually or use a custom rviz.
    # For now, we let Fast-LIO launch its RViz.

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        tron1_control,
        TimerAction(period=2.0, actions=[fast_lio_slam]), # Wait for TF/Robot
        TimerAction(period=5.0, actions=[nav2_navigation]), # Wait for SLAM
    ])

