"""
Launch file for the Kinematic Bicycle simulation stack.
Processes racecar.xacro, launches robot_state_publisher, simulation plant (bicycle_sim),
path generator and lap analyzer (track_environment), RViz2, and controller (bicycle_control).
"""

import os
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('bicycle_sim')
        rviz_config = os.path.join(pkg_share, 'bicycle.rviz')
        urdf_file = os.path.join(pkg_share, 'urdf', 'racecar.urdf')
    except (PackageNotFoundError, Exception):
        pkg_share = None
        rviz_config = None
        urdf_file = None

    # Fallback to local source path if package is not yet installed in share
    if not urdf_file or not os.path.isfile(urdf_file):
        urdf_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'racecar.urdf')
        )
    if not rviz_config or not os.path.isfile(rviz_config):
        rviz_config = os.path.abspath(
            os.path.join(os.path.dirname(__file__), 'bicycle.rviz')
        )

    # Launch configuration variables
    use_rviz = LaunchConfiguration('rviz')
    track_file = LaunchConfiguration('track_file')
    trajectory_type = LaunchConfiguration('trajectory_type')
    controller = LaunchConfiguration('controller')
    use_analyzer = LaunchConfiguration('analyzer')

    # Load URDF XML directly without requiring external xacro CLI
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
    else:
        xacro_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'racecar.xacro')
        )
        try:
            import xacro
            robot_description = xacro.process_file(xacro_file).toxml()
        except Exception:
            robot_description = ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'track_file',
            default_value='centerline_0.csv',
            description='CSV track file to load for the path and simulator start pose'
        ),
        DeclareLaunchArgument(
            'trajectory_type',
            default_value='centerline',
            description='Trajectory type to load: centerline, sp, or iqp'
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='none',
            description='Controller to launch: none, pure_pursuit, lateral_pid, mpc, teleop'
        ),
        DeclareLaunchArgument(
            'analyzer',
            default_value='true',
            description='Launch lap analyzer and real-time HUD'
        ),
        DeclareLaunchArgument(
            'use_cruise_control',
            default_value='false',
            description='Enable closed-loop longitudinal cruise control in teleop bridge'
        ),

        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 2. Kinematic Bicycle Simulator Plant Node (from bicycle_sim)
        Node(
            package='bicycle_sim',
            executable='sim_node',
            name='kinematic_bicycle',
            output='screen',
            arguments=['--track', track_file],
            parameters=[{
                'wheelbase_length': 1.25,
                'dt': 0.1,
                'car_name': 'ego_racecar'
            }]
        ),

        # 3. Path Generator Node (from track_environment)
        Node(
            package='track_environment',
            executable='path_gen',
            name='path_gen',
            output='screen',
            parameters=[{
                'track_file': track_file,
                'trajectory_type': trajectory_type,
                'close_loop': True
            }]
        ),

        # 4. Lap Analyzer & Performance Monitor (from track_environment)
        Node(
            package='track_environment',
            executable='lap_analyzer',
            name='lap_analyzer',
            output='screen',
            condition=IfCondition(use_analyzer)
        ),

        # 5. RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz)
        ),

        # 6. Controllers from bicycle_control
        # Mode A: Lateral PID + Longitudinal PID
        Node(
            package='bicycle_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'control_mode': 'lateral_pid',
                'target_speed': 4.0,
                'velocity_mode': 'curvature'
            }],
            condition=IfCondition(
                PythonExpression(
                    ["'", controller, "'.lower() == 'lateral_pid'"]
                )
            )
        ),

        # Mode B: Pure Pursuit + Longitudinal PID
        Node(
            package='bicycle_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'control_mode': 'pure_pursuit',
                'target_speed': 4.0,
                'velocity_mode': 'curvature'
            }],
            condition=IfCondition(
                PythonExpression(
                    ["'", controller, "'.lower() == 'pure_pursuit'"]
                )
            )
        ),

        # Mode C: Kinematic MPC
        Node(
            package='bicycle_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'control_mode': 'mpc',
                'target_speed': 4.0
            }],
            condition=IfCondition(
                PythonExpression(
                    ["'", controller, "'.lower() == 'mpc'"]
                )
            )
        ),

        # Mode D: Keyboard Teleoperation Bridge
        Node(
            package='bicycle_control',
            executable='teleop_bridge',
            name='teleop_bridge',
            output='screen',
            parameters=[{
                'use_cruise_control': LaunchConfiguration('use_cruise_control')
            }],
            condition=IfCondition(
                PythonExpression(
                    ["'", controller, "'.lower() == 'teleop'"]
                )
            )
        ),
    ])
