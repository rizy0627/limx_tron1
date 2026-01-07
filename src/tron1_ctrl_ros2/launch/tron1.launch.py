from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tron1_ctrl_ros2',
            executable='tron1_node',
            name='tron1_node',
            output='screen',
            parameters=[
                # {'server_uri': 'ws://127.0.0.1:5000'}, # 修改为本地回环地址
                {'server_uri': 'ws://10.192.1.2:5000'}, # 机器人路径
                {'accid': 'WF_TRON1A_519'},
                {'max_linear_x': 1.0},
                {'max_linear_y': 1.0},
                {'max_angular_z': 1.0}
            ]
        )
    ])
