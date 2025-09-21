from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_driver',
            executable='lift_driver_node',
            name='lift_driver_node',
            output='screen',
            # 如果你给 C++ 节点加了串口参数，可在此传入：
            # parameters=[{'port': '/dev/ttyUSB0', 'baudrate': 38400}],
        ),
        Node(
            package='lift_driver',
            executable='teleop_lift_key.py',
            name='lift_teleop_key',
            output='screen',
            parameters=[{
                'repeat_rate': 10.0,
                'idle_stop_timeout': 0.2,
            }],
        ),
    ])
