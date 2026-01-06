from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Ensure ROBOT_TYPE is set for pointfoot_gazebo
    if 'ROBOT_TYPE' not in os.environ:
        print("ROBOT_TYPE not set, defaulting to WF_TRON1B")
        os.environ['ROBOT_TYPE'] = 'WF_TRON1B'

    pointfoot_gazebo_dir = FindPackageShare('pointfoot_gazebo')

    # 1. Simulation Launch (Empty World)
    # This launches Gazebo and spawns the robot
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pointfoot_gazebo_dir, 'launch', 'empty_world.launch.py'])
        )
    )

    # 2. Bridge Node (mrosbridger)
    # Acts as a bridge between the SDK (used by Sim) and WebSockets (used by Control)
    # Delayed to ensure Gazebo is fully ready and listening
    bridge_node = Node(
        package='mrosbridger',
        executable='mrosbridger',
        name='mrosbridger',
        output='screen',
        additional_env={'MROS_AGENT_IP': '127.0.0.1'},
        parameters=[
            {"bridge_mros2ros": True},
            {"mros2ros_include": "/tf;/tf_static;/odom"}
        ]
    )

    delayed_bridge = TimerAction(
        period=5.0, # Wait 5s for Sim
        actions=[bridge_node]
    )

    # 3. Tron1 Control Node
    # Connects to the WebSocket server exposed by mrosbridger
    # Delayed further to ensure Bridge is connected and listening
    ctrl_node = Node(
        package='tron1_ctrl_ros2',
        executable='tron1_node',
        name='tron1_node',
        output='screen',
        parameters=[{
            'server_uri': 'ws://127.0.0.1:5000',
            'max_linear_x': 1.0,
            'max_linear_y': 1.0,
            'max_angular_z': 1.0
        }]
    )

    delayed_ctrl = TimerAction(
        period=8.0, # Wait 8s total (Bridge gets 3s to initialize)
        actions=[ctrl_node]
    )

    return LaunchDescription([
        sim_launch,
        delayed_bridge,
        delayed_ctrl
    ])
