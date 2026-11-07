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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def setup_trajectory_generator_node(context, *args, **kwargs) -> list[Node]:
    """Dynamically configure trajectory_generator_node with YAML and CLI overrides."""
    config_path = LaunchConfiguration('trajectory_config').perform(context)

    node_params: list[object] = []
    if config_path and os.path.exists(config_path):
        node_params.append(config_path)

    cli_overrides: dict[str, object] = {}
    robot_id_val = int(LaunchConfiguration('robot_id').perform(context))
    cli_overrides['robot_id'] = robot_id_val

    param_converters = {
        'pattern': str,
        'center_x': float,
        'center_y': float,
        'total_points': int,
        'path_window_size': int,
        'lookahead_steps': int,
        'face_center': lambda v: v.lower() in ('true', '1', 'yes'),
        'circle_radius': float,
        'circle_center_x': float,
        'circle_center_y': float,
        'square_side': float,
        'square_center_x': float,
        'square_center_y': float,
        'wave_amplitude': float,
        'wave_step_length': float,
        'wave_num_cycles': int,
        'wave_center_x': float,
        'wave_center_y': float,
        'length': float,
        'x_initial': float,
        'y_initial': float,
        'line_angle': float,
    }

    for param_name, converter in param_converters.items():
        val = LaunchConfiguration(param_name).perform(context)
        if val != '':
            try:
                cli_overrides[param_name] = converter(val)
            except Exception:
                cli_overrides[param_name] = val

    if cli_overrides:
        node_params.append(cli_overrides)

    return [
        Node(
            package='oxebots_trajectory_analysis',
            executable='trajectory_generator_node',
            name='trajectory_generator_node',
            output='screen',
            parameters=node_params,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the complete ROS 2 launch description for trajectory benchmarking.

    Returns:
        LaunchDescription: Orchestrated collection of nodes, configurations, and arguments.
    """
    bringup_pkg_share = get_package_share_directory('oxebots_bringup')
    default_config_file = os.path.join(bringup_pkg_share, 'config', 'match_config.yaml')

    analysis_pkg_share = get_package_share_directory('oxebots_trajectory_analysis')
    src_config_file = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'config', 'trajectory_config.yaml'
    ))
    default_trajectory_config = (
        src_config_file if os.path.exists(src_config_file)
        else os.path.join(analysis_pkg_share, 'config', 'trajectory_config.yaml')
    )

    # Primary operational arguments
    declare_trajectory_config = DeclareLaunchArgument(
        'trajectory_config',
        default_value=default_trajectory_config,
        description='Path to the trajectory YAML configuration file.',
    )

    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='ID of the target robot under test and analysis.',
    )

    declare_pattern = DeclareLaunchArgument(
        'pattern',
        default_value='',
        description='Geometric trajectory pattern: "circle", "square", "square_wave", or "line" (reads from YAML if empty).',
    )

    declare_center_x = DeclareLaunchArgument(
        'center_x',
        default_value='',
        description='Global trajectory center X coordinate in meters (reads from YAML if empty).',
    )

    declare_center_y = DeclareLaunchArgument(
        'center_y',
        default_value='',
        description='Global trajectory center Y coordinate in meters (reads from YAML if empty).',
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
        default_value='',
        description='Number of steps ahead in the reference array (reads from YAML if empty).',
    )

    declare_total_points = DeclareLaunchArgument(
        'total_points',
        default_value='',
        description='Total number of discrete nodes along the geometric reference path (reads from YAML if empty).',
    )

    declare_path_window_size = DeclareLaunchArgument(
        'path_window_size',
        default_value='',
        description='Sliding window size of waypoints published to the path follower (reads from YAML if empty).',
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
        default_value='',
        description='Whether to keep the robot heading oriented toward the trajectory center (reads from YAML if empty).',
    )

    # Parametric geometry definitions (all read from YAML if empty)
    declare_circle_radius = DeclareLaunchArgument('circle_radius', default_value='', description='Radius of the circle in meters.')
    declare_circle_center_x = DeclareLaunchArgument('circle_center_x', default_value='', description='Center X coordinate of the circle.')
    declare_circle_center_y = DeclareLaunchArgument('circle_center_y', default_value='', description='Center Y coordinate of the circle.')

    declare_line_length = DeclareLaunchArgument('length', default_value='', description='Length in meters of line.')
    declare_line_initial_x = DeclareLaunchArgument('x_initial', default_value='', description='Position x initial in meters of line.')
    declare_line_initial_y = DeclareLaunchArgument('y_initial', default_value='', description='Position y initial in meters of line.')
    declare_line_angle = DeclareLaunchArgument('line_angle', default_value='', description='Angle in radians of line.')

    declare_square_side = DeclareLaunchArgument('square_side', default_value='', description='Side length of the square in meters.')
    declare_square_center_x = DeclareLaunchArgument('square_center_x', default_value='', description='Center X coordinate of the square.')
    declare_square_center_y = DeclareLaunchArgument('square_center_y', default_value='', description='Center Y coordinate of the square.')

    declare_wave_amp = DeclareLaunchArgument('wave_amplitude', default_value='', description='Y amplitude of the square wave.')
    declare_wave_step = DeclareLaunchArgument('wave_step_length', default_value='', description='X step length for each wave transition.')
    declare_wave_cycles = DeclareLaunchArgument('wave_num_cycles', default_value='', description='Total complete wave cycles.')
    declare_wave_center_x = DeclareLaunchArgument('wave_center_x', default_value='', description='Center X coordinate of the square wave.')
    declare_wave_center_y = DeclareLaunchArgument('wave_center_y', default_value='', description='Center Y coordinate of the square wave.')

    # Configuration substitutions
    robot_id = LaunchConfiguration('robot_id')
    run_plotter = LaunchConfiguration('run_plotter')
    record_bag = LaunchConfiguration('record_bag')
    standalone = LaunchConfiguration('standalone')
    target = LaunchConfiguration('target')
    use_movement_calculation = LaunchConfiguration('use_movement_calculation')
    margin_mm = LaunchConfiguration('margin_mm')
    lookahead_mm = LaunchConfiguration('lookahead_mm')
    p_gain_linear = LaunchConfiguration('p_gain_linear')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    p_gain_angular = LaunchConfiguration('p_gain_angular')

    # Real-Time Matplotlib Visualization Node
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

    # Auxiliary Nodes for Standalone Execution
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
                'path_renews_watchdog': True,
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

    # Optional Rosbag Recording Process
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
        declare_trajectory_config,
        declare_robot_id,
        declare_pattern,
        declare_center_x,
        declare_center_y,
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
        declare_line_length,
        declare_line_initial_x,
        declare_line_initial_y,
        declare_line_angle,
        OpaqueFunction(function=setup_trajectory_generator_node),
        plotter_node,
        standalone_vision,
        standalone_observer,
        standalone_grsim_comms,
        standalone_nrf24_comms,
        standalone_follower,
        rosbag_record,
    ])
