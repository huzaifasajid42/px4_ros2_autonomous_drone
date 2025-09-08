from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    hold_time_arg = DeclareLaunchArgument(
        'hold_time',
        default_value='3.0',
        description='Time to hold at each waypoint (seconds)'
    )
    
    mission_alt_arg = DeclareLaunchArgument(
        'mission_altitude',
        default_value='5.0',
        description='Mission altitude (meters) for all waypoints'
    )

    # 1. MAVROS NODE - using Node action with parameters dict (not ExecuteProcess)
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',  # Empty string in parameters dict is fine
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0'
        }],
        arguments=['--log-level', 'debug']  # Add debug logging
    )

    # 2. YOUR MISSION CONTROLLER NODE
    mission_controller_node = Node(
        package='drone_controller',
        executable='drone_controller_node',
        name='drone_controller_node',
        output='screen',
        parameters=[{
            'hold_time': LaunchConfiguration('hold_time'),
            'mission_altitude': LaunchConfiguration('mission_altitude'),
            'waypoints': [
                ['0.0', '0.0', '5.0'],
                ['5.0', '0.0', '5.0'],
                ['5.0', '5.0', '5.0'],
                ['0.0', '5.0', '5.0'],
                ['0.0', '0.0', '5.0']
            ]
        }]
    )

    # 3. Delay the mission controller to let MAVROS stabilize
    delayed_mission_controller = TimerAction(
        period=15.0,
        actions=[mission_controller_node]
    )

    return LaunchDescription([
        hold_time_arg,
        mission_alt_arg,
        mavros_node,
        delayed_mission_controller,
    ])