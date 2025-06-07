from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboguard_estop_package',
            executable='estop_hardware_monitor',
            name='estop_hardware_monitor'
        ),
        Node(
            package='roboguard_estop_package',
            executable='estop_heartbeat_unifier',
            name='estop_heartbeat_unifier'
        ),
        Node(
            package='roboguard_estop_package',
            executable='estop_manager',
            name='estop_manager'
        ),
        Node(
            package='roboguard_estop_package',
            executable='roboguard_powersupply_toggler',
            name='roboguard_powersupply_toggler'
        ),
    ])
