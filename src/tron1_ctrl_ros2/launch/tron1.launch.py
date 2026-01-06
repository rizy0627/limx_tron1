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
                {'server_uri': 'ws://127.0.0.1:5000'}, # 修改为本地回环地址
                # {'accid': 'PF_TRON1A_xxx'},
                {'max_linear_x': 1.0},
                {'max_linear_y': 1.0},
                {'max_angular_z': 1.0}
            ],
            # 关键：将订阅的 cmd_vel 重映射为 /user/cmd_vel
            # 这样它就不会收到 bridge 发出的 /cmd_vel，避免死循环
            remappings=[
                ('/cmd_vel', '/user/cmd_vel')
            ]
        )
    ])
