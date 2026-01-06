from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='IP address of the robot (127.0.0.1 for sim, 10.192.1.2 for real)'
    )
    
    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='false',
        description='Enable internal keyboard control'
    )

    return LaunchDescription([
        robot_ip_arg,
        use_keyboard_arg,
        Node(
            package='tron1_ctrl_ros2',
            executable='limxsdk_teleop',
            name='limxsdk_teleop_node',
            output='screen',
            parameters=[
                {'robot_ip': LaunchConfiguration('robot_ip')},
                {'use_keyboard': LaunchConfiguration('use_keyboard')}
            ],
            # If using keyboard, we need input/output access
            # 'screen' output is already set, but for input we might need prefix
            # usually running in a separate terminal is better, but this works for basic cases
            prefix=['xterm -e'] if LaunchConfiguration('use_keyboard') == 'true' else [] 
            # Note: prefix logic in Python launch files is tricky with conditions.
            # Simplified: Just run node, user types in that terminal.
        )
    ])
