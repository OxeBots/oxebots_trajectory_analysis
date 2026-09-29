"""Launch description for analytical trajectory benchmarking and evaluation.

This launch file coordinates the analytical trajectory generator, real-time
Matplotlib visualization node, and optional standalone infrastructure nodes
within the RoboCup Small Size League (SSL) environment.

Supports three operational pipelines:
    - Mode 1 (Simulation): Generator -> movement_calculation_node -> grSim_controller_node -> grSim.
    - Mode 2 (Real Robot Direct Points): Generator -> direct /robot_goal & /robot_<id>/path -> Real Robot.
    - Mode 3 (Real Robot PC Follower): Generator -> movement_calculation_node -> /robot_commands -> nrf24_bridge_node -> Real Robot.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the complete ROS 2 launch description for trajectory benchmarking.

    Returns:
        LaunchDescription: Orchestrated collection of nodes, configurations, and arguments.
    """
    bringup_pkg_share = get_package_share_directory('oxebots_bringup')
    default_config_file = os.path.join(bringup_pkg_share, 'config', 'match_config.yaml')

    # Primary operational arguments
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='ID of the target robot under test and analysis.',
    )

    declare_pattern = DeclareLaunchArgument(
        'pattern',
        default_value='circle',
        description='Geometric trajectory pattern: "circle", "square", or "square_wave".',
    )

    declare_run_plotter = DeclareLaunchArgument(
        'run_plotter',
        default_value='true',
        description='Whether to launch the real-time Matplotlib visualizer.',
    )

    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Whether to record test topics to a rosbag dataset.',
    )

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to spin up vision, comms, and follower in standalone mode.',
    )

    declare_target = DeclareLaunchArgument(
        'target',
        default_value='sim',
        description='Execution target: "sim" (grSim simulation) or "real" (physical robot via nRF24 radio).',
    )

    declare_use_movement_calculation = DeclareLaunchArgument(
        'use_movement_calculation',
        default_value='auto',
        description=(
            'Pipeline routing through movement_calculation_node: '
            '"auto" (true for sim, false for real), '
            '"true" (Mode 1 / Mode 3: follower on PC -> /robot_commands), or '
            '"false" (Mode 2: bypass PC follower, publish direct points).'
        ),
    )

    declare_margin_mm = DeclareLaunchArgument(
        'margin_mm',
        default_value='250.0',
        description='Viewport margin around the trajectory contour in millimeters.',
    )

    declare_lookahead_mm = DeclareLaunchArgument(
        'lookahead_mm',
        default_value='45.0',
        description='Pure pursuit lookahead distance in millimeters.',
    )

    declare_lookahead_steps = DeclareLaunchArgument(
        'lookahead_steps',
        default_value='2',
        description='Number of steps ahead in the reference array (~33 mm per step with 150 points).',
    )

    declare_total_points = DeclareLaunchArgument(
        'total_points',
        default_value='150',
        description='Total number of discrete nodes along the geometric reference path.',
    )

    declare_path_window_size = DeclareLaunchArgument(
        'path_window_size',
        default_value='15',
        description='Sliding window size of waypoints published to the path follower.',
    )

    declare_p_gain_linear = DeclareLaunchArgument(
        'p_gain_linear',
        default_value='3.0',
        description='Proportional linear velocity gain for path following.',
    )

    declare_max_linear_speed = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.20',
        description='Maximum linear velocity limit in m/s.',
    )

    declare_max_angular_speed = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.5',
        description='Maximum angular velocity limit in rad/s.',
    )

    declare_p_gain_angular = DeclareLaunchArgument(
        'p_gain_angular',
        default_value='2.0',
        description='Proportional angular velocity gain.',
    )

    declare_face_center = DeclareLaunchArgument(
        'face_center',
        default_value='true',
        description='Whether to keep the robot heading oriented toward the trajectory center.',
    )

    # Parametric geometry definitions
    declare_circle_radius = DeclareLaunchArgument('circle_radius', default_value='0.8', description='Radius of the circle in meters.')
    declare_circle_center_x = DeclareLaunchArgument('circle_center_x', default_value='0.0', description='Center X coordinate of the circle.')
    declare_circle_center_y = DeclareLaunchArgument('circle_center_y', default_value='0.0', description='Center Y coordinate of the circle.')

    declare_square_side = DeclareLaunchArgument('square_side', default_value='1.4', description='Side length of the square in meters.')
    declare_square_center_x = DeclareLaunchArgument('square_center_x', default_value='0.0', description='Center X coordinate of the square.')
    declare_square_center_y = DeclareLaunchArgument('square_center_y', default_value='0.0', description='Center Y coordinate of the square.')

    declare_wave_amp = DeclareLaunchArgument('wave_amplitude', default_value='0.5', description='Y amplitude of the square wave.')
    declare_wave_step = DeclareLaunchArgument('wave_step_length', default_value='0.6', description='X step length for each wave transition.')
    declare_wave_cycles = DeclareLaunchArgument('wave_num_cycles', default_value='2', description='Total complete wave cycles.')
    declare_wave_center_x = DeclareLaunchArgument('wave_center_x', default_value='0.0', description='Center X coordinate of the square wave.')
    declare_wave_center_y = DeclareLaunchArgument('wave_center_y', default_value='0.0', description='Center Y coordinate of the square wave.')

    # Configuration substitutions
    robot_id = LaunchConfiguration('robot_id')
    pattern = LaunchConfiguration('pattern')
    run_plotter = LaunchConfiguration('run_plotter')
    record_bag = LaunchConfiguration('record_bag')
    standalone = LaunchConfiguration('standalone')
    target = LaunchConfiguration('target')
    use_movement_calculation = LaunchConfiguration('use_movement_calculation')
    margin_mm = LaunchConfiguration('margin_mm')
    lookahead_mm = LaunchConfiguration('lookahead_mm')
    lookahead_steps = LaunchConfiguration('lookahead_steps')
    total_points = LaunchConfiguration('total_points')
    path_window_size = LaunchConfiguration('path_window_size')
    p_gain_linear = LaunchConfiguration('p_gain_linear')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    p_gain_angular = LaunchConfiguration('p_gain_angular')
    face_center = LaunchConfiguration('face_center')



    circle_radius = LaunchConfiguration('circle_radius')
    circle_center_x = LaunchConfiguration('circle_center_x')
    circle_center_y = LaunchConfiguration('circle_center_y')
    square_side = LaunchConfiguration('square_side')
    square_center_x = LaunchConfiguration('square_center_x')
    square_center_y = LaunchConfiguration('square_center_y')
    wave_amplitude = LaunchConfiguration('wave_amplitude')
    wave_step_length = LaunchConfiguration('wave_step_length')
    wave_num_cycles = LaunchConfiguration('wave_num_cycles')
    wave_center_x = LaunchConfiguration('wave_center_x')
    wave_center_y = LaunchConfiguration('wave_center_y')

    #Analytical Trajectory Generator Node
    analytic_gen_node = Node(
        package='oxebots_trajectory_analysis',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'pattern': pattern,
            'total_points': total_points,
            'path_window_size': path_window_size,
            'lookahead_steps': lookahead_steps,
            'face_center': face_center,
            'circle_radius': circle_radius,
            'circle_center_x': circle_center_x,
            'circle_center_y': circle_center_y,
            'square_side': square_side,
            'square_center_x': square_center_x,
            'square_center_y': square_center_y,
            'wave_amplitude': wave_amplitude,
            'wave_step_length': wave_step_length,
            'wave_num_cycles': wave_num_cycles,
            'wave_center_x': wave_center_x,
            'wave_center_y': wave_center_y,
        }],
    )

    #Real-Time Matplotlib Visualization Node
    plotter_node = Node(
        package='oxebots_trajectory_analysis',
        executable='trajectory_plotter_node',
        name='trajectory_plotter_node',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'margin_mm': margin_mm,
            'update_interval_ms': 50,
        }],
        condition=IfCondition(run_plotter),
    )

    #Auxiliary Nodes for Standalone Execution
    standalone_vision = Node(
        package='ssl_ros_bridge',
        executable='vision_bridge_node',
        name='ssl_vision_bridge',
        parameters=[default_config_file],
        output='screen',
        condition=IfCondition(standalone),
    )

    standalone_observer = Node(
        package='oxebots_observers',
        executable='game_observer_node',
        name='game_observer_node',
        parameters=[default_config_file],
        output='screen',
        condition=IfCondition(standalone),
    )

    # grSim simulator communications
    standalone_grsim_comms = Node(
        package='oxebots_comms',
        executable='grSim_controller_node',
        name='grSim_controller_node',
        parameters=[default_config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'true' if ('", standalone, "' == 'true' and '", target, "' in ('sim', 'simulation', 'grsim')) else 'false'"
            ])
        ),
    )

    # Physical robot radio bridge communications (nRF24L01+)
    standalone_nrf24_comms = Node(
        package='oxebots_comms',
        executable='nrf24_bridge_node',
        name='nrf24_hardware_bridge',
        parameters=[default_config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'true' if ('", standalone, "' == 'true' and '", target, "' in ('real', 'real_robot', 'robot', 'hardware')) else 'false'"
            ])
        ),
    )

    # Path follower node
    standalone_follower = Node(
        package='oxebots_strategy',
        executable='movement_calculation_node',
        name=PythonExpression(["'path_follower_node_' + str('", robot_id, "')"]),
        parameters=[
            default_config_file,
            {
                'robot_id': robot_id,
                'lookahead_distance': lookahead_mm,
                'p_gain_linear': p_gain_linear,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
                'p_gain_angular': p_gain_angular,
            }
        ],
        remappings=[
            ('/robot_goal', '/unused_benchmark_goal')
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'true' if ('", standalone, "' == 'true' and ('",
                use_movement_calculation, "' == 'true' or ('",
                use_movement_calculation, "' == 'auto' and '", target, "' in ('sim', 'simulation', 'grsim')))) else 'false'"
            ])
        ),
    )

    #Optional Rosbag Recording Process
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/game_data',
            '/robot_goal',
            PythonExpression(["'/robot_' + str('", robot_id, "') + '/path'"]),
            PythonExpression(["'/robot_' + str('", robot_id, "') + '/path_viz'"]),
            '/robot_commands',
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        declare_robot_id,
        declare_pattern,
        declare_run_plotter,
        declare_record_bag,
        declare_standalone,
        declare_target,
        declare_use_movement_calculation,
        declare_margin_mm,
        declare_lookahead_mm,
        declare_lookahead_steps,
        declare_total_points,
        declare_path_window_size,
        declare_p_gain_linear,
        declare_max_linear_speed,
        declare_max_angular_speed,
        declare_p_gain_angular,
        declare_face_center,
        declare_circle_radius,
        declare_circle_center_x,
        declare_circle_center_y,
        declare_square_side,
        declare_square_center_x,
        declare_square_center_y,
        declare_wave_amp,
        declare_wave_step,
        declare_wave_cycles,
        declare_wave_center_x,
        declare_wave_center_y,
        analytic_gen_node,
        plotter_node,
        standalone_vision,
        standalone_observer,
        standalone_grsim_comms,
        standalone_nrf24_comms,
        standalone_follower,
        rosbag_record,
    ])
