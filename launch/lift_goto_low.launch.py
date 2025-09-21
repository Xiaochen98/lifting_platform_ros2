from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_driver',
            executable='lift_driver_node',
            name='lift_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 38400
            }]
        ),
        # 一键降到最低
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/lift/cmd', 'std_msgs/String', '{data: "down"}'],
            output='screen'
        )
    ])
