from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_uav_qgc',
            arguments=[
                '--standalone', 'rqt_uav_qgc',
                '--force-discover',
                '--clear-config'
            ],
            output='screen'
        )
    ])
