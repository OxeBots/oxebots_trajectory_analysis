#!/usr/bin/env python3
"""Real-time Matplotlib visualization and tracking error quantification for SSL.

This module provides the TrajectoryPlotterNode, which visualizes the robot's
real-time trajectory against the discrete analytical reference path within the
RoboCup Small Size League (SSL) environment. The visualizer focuses exclusively
on the active geometric bounding box (with millimeter scaling), calculates
cross-track errors continuously, and reports comprehensive statistical metrics
(MAE, RMSE, Standard Deviation, Peak Error) upon test completion.

Academic standards applied:
    - Cross-track Euclidean error calculation: e_k = min_j ||p_robot(t_k) - p_ref,j||.
    - Root Mean Square Error (RMSE) and Mean Absolute Error (MAE) reporting.
    - High-resolution (300 DPI) vector/raster image archiving for publications.
    - PEP 8 (Style Guide for Python Code) and PEP 257 (Docstring Conventions).
"""

from __future__ import annotations

import datetime
import math
import os
import threading
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from oxebots_interfaces.msg import GameData, RobotGoal


def get_default_image_dir() -> str:
    """Determine the default filesystem directory for saving trajectory plots.

    Resolves dynamically across different environments (source directory,
    colcon workspaces, devcontainers, or user environments) to ensure
    persistent figure storage without machine-specific hardcoded paths.

    Returns:
        str: Absolute directory path for output figure storage.
    """
    env_dir = os.environ.get('OXEBOTS_TRAJECTORY_IMAGE_DIR')
    if env_dir:
        return os.path.abspath(env_dir)

    curr = os.path.abspath(__file__)

    # Case 1: Running from colcon build/install directories -> redirect to src/
    for sub in ['/build/oxebots_trajectory_analysis', '/install/oxebots_trajectory_analysis']:
        if sub in curr:
            ws_root = curr.split(sub)[0]
            candidate = os.path.join(ws_root, 'src', 'oxebots_trajectory_analysis', 'imagens')
            if os.path.exists(os.path.dirname(candidate)):
                return candidate

    # Case 2: Running directly from source tree
    pkg_src_dir = os.path.dirname(os.path.dirname(curr))
    if os.path.basename(pkg_src_dir) == 'oxebots_trajectory_analysis':
        candidate = os.path.join(pkg_src_dir, 'imagens')
        if os.path.exists(candidate) or os.path.exists(pkg_src_dir):
            return candidate

    # Case 3: Check colcon / ament environment prefixes
    for env_var in ['COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH']:
        prefixes = os.environ.get(env_var, '').split(os.pathsep)
        for prefix in prefixes:
            if not prefix:
                continue
            ws_candidate = prefix
            while ws_candidate and os.path.basename(ws_candidate) in ['install', 'build', 'oxebots_trajectory_analysis']:
                ws_candidate = os.path.dirname(ws_candidate)
            candidate = os.path.join(ws_candidate, 'src', 'oxebots_trajectory_analysis', 'imagens')
            if os.path.exists(os.path.dirname(candidate)):
                return candidate

    # Case 4: Standard container workspace or current working directory
    for root in [os.getcwd(), '/software_ws']:
        candidate = os.path.join(root, 'src', 'oxebots_trajectory_analysis', 'imagens')
        if os.path.exists(os.path.dirname(candidate)):
            return candidate
        cwd_imagens = os.path.join(root, 'imagens')
        if os.path.exists(cwd_imagens):
            return cwd_imagens

    return os.path.join(os.getcwd(), 'imagens')


class TrajectoryPlotterNode(Node):
    """ROS 2 Node for real-time Matplotlib rendering and statistical error logging.

    Attributes:
        robot_id (int): Identifier of the tracked robot.
        interval_ms (int): Matplotlib animation update frequency in milliseconds.
        margin_mm (float): Viewport margin padding around the reference contour in mm.
        image_dir (str): Directory path where generated trajectory plots are archived.
    """

    def __init__(self) -> None:
        """Initialize parameters, communication subscriptions, and telemetry buffers."""
        super().__init__('trajectory_plotter_node')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('update_interval_ms', 50)
        self.declare_parameter('margin_mm', 250.0)
        default_img_dir = get_default_image_dir()
        self.declare_parameter('image_dir', default_img_dir)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.interval_ms = self.get_parameter('update_interval_ms').get_parameter_value().integer_value
        self.margin_mm = self.get_parameter('margin_mm').get_parameter_value().double_value
        configured_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        self.image_dir = configured_dir if configured_dir else default_img_dir

        self.lock = threading.Lock()
        self.summary_printed = False
        self.fig: plt.Figure | None = None

        # Discretized planned trajectory coordinates (in millimeters)
        self.plan_x_mm: list[float] = []
        self.plan_y_mm: list[float] = []

        # Continuous robot position history (in millimeters)
        self.real_x_mm: list[float] = []
        self.real_y_mm: list[float] = []
        self.current_x_mm: float | None = None
        self.current_y_mm: float | None = None
        self.current_yaw: float = 0.0

        # Active target goal coordinates (in millimeters)
        self.goal_x_mm: float | None = None
        self.goal_y_mm: float | None = None

        # Cross-track Euclidean error tracking history (in millimeters)
        self.error_history_mm: list[float] = []
        self.time_history: list[float] = []
        self.start_time = self.get_clock().now()

        # Dynamic viewport bounding coordinates
        self.bounds_set = False
        self.min_x = -1000.0
        self.max_x = 1000.0
        self.min_y = -1000.0
        self.max_y = 1000.0

        # Topic subscriptions
        robot_prefix = f'/robot_{self.robot_id}'
        self.path_viz_received = False
        self.viz_sub = self.create_subscription(
            Path, f'{robot_prefix}/path_viz', self.path_cb, 10
        )
        self.path_sub = self.create_subscription(
            Path, f'{robot_prefix}/path', self.path_fallback_cb, 10
        )
        self.game_data_sub = self.create_subscription(
            GameData, '/game_data', self.game_data_cb, 10
        )
        self.goal_sub = self.create_subscription(
            RobotGoal, f'{robot_prefix}/target_goal', self.goal_cb, 10
        )
        self.fallback_goal_sub = self.create_subscription(
            RobotGoal, '/robot_goal', self.goal_cb, 10
        )

        self.get_logger().info(
            f'Trajectory Plotter initialized for robot {self.robot_id} '
            f'(Measurement Unit: mm | Viewport Margin: {self.margin_mm:.0f} mm).'
        )

    def path_cb(self, msg: Path) -> None:
        """Process complete static reference trajectory from /path_viz.

        Args:
            msg: Path message containing the full set of reference waypoints in meters.
        """
        with self.lock:
            self.path_viz_received = True
            # Convert waypoint coordinates from meters to millimeters
            self.plan_x_mm = [p.pose.position.x * 1000.0 for p in msg.poses]
            self.plan_y_mm = [p.pose.position.y * 1000.0 for p in msg.poses]

            if self.plan_x_mm and self.plan_y_mm:
                self.min_x = min(self.plan_x_mm) - self.margin_mm
                self.max_x = max(self.plan_x_mm) + self.margin_mm
                self.min_y = min(self.plan_y_mm) - self.margin_mm
                self.max_y = max(self.plan_y_mm) + self.margin_mm
                self.bounds_set = True

    def path_fallback_cb(self, msg: Path) -> None:
        """Process active path fallback when /path_viz has not yet been received.

        Args:
            msg: Path message containing active waypoints in meters.
        """
        with self.lock:
            if not self.path_viz_received and not self.plan_x_mm and msg.poses:
                self.plan_x_mm = [p.pose.position.x * 1000.0 for p in msg.poses]
                self.plan_y_mm = [p.pose.position.y * 1000.0 for p in msg.poses]
                self.min_x = min(self.plan_x_mm) - self.margin_mm
                self.max_x = max(self.plan_x_mm) + self.margin_mm
                self.min_y = min(self.plan_y_mm) - self.margin_mm
                self.max_y = max(self.plan_y_mm) + self.margin_mm
                self.bounds_set = True

    def goal_cb(self, msg: RobotGoal) -> None:
        """Process active target goal updates.

        Args:
            msg: RobotGoal message containing the current target pose in meters.
        """
        if msg.robot_id == self.robot_id:
            with self.lock:
                self.goal_x_mm = msg.pose.pose.position.x * 1000.0
                self.goal_y_mm = msg.pose.pose.position.y * 1000.0

    def game_data_cb(self, msg: GameData) -> None:
        """Receive vision telemetry and compute real-time cross-track Euclidean error.

        Args:
            msg: GameData message containing global robot coordinates (in millimeters).
        """
        robot = None
        for ally in msg.robots.allies:
            if ally.id == self.robot_id:
                robot = ally
                break
        if robot is None:
            for enemy in msg.robots.enemies:
                if enemy.id == self.robot_id:
                    robot = enemy
                    break

        if robot is not None:
            rx_mm = float(robot.x)
            ry_mm = float(robot.y)
            orientation = float(robot.orientation)

            with self.lock:
                self.current_x_mm = rx_mm
                self.current_y_mm = ry_mm
                self.current_yaw = orientation
                self.real_x_mm.append(rx_mm)
                self.real_y_mm.append(ry_mm)

                # Compute minimum Euclidean distance to reference contour (cross-track error)
                if self.plan_x_mm and self.plan_y_mm:
                    dists = [
                        math.hypot(px - rx_mm, py - ry_mm)
                        for px, py in zip(self.plan_x_mm, self.plan_y_mm)
                    ]
                    min_err_mm = min(dists)
                else:
                    min_err_mm = 0.0

                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.time_history.append(elapsed)
                self.error_history_mm.append(min_err_mm)

    def print_error_summary(self) -> None:
        """Calculate and print comprehensive statistical error metrics to the console.

        Computes Mean Absolute Error (MAE), Root Mean Square Error (RMSE),
        Standard Deviation, and Peak Errors, and archives the plot to disk at 300 DPI.
        """
        with self.lock:
            if self.summary_printed:
                return
            self.summary_printed = True

            if not self.error_history_mm:
                print('\n[oxebots_trajectory_analysis] No error samples recorded during the trial.')
                return

            errs = np.array(self.error_history_mm)
            total_time = self.time_history[-1] if self.time_history else 0.0

        mae = float(np.mean(errs))
        rmse = float(np.sqrt(np.mean(np.square(errs))))
        max_err = float(np.max(errs))
        min_err = float(np.min(errs))
        std_err = float(np.std(errs))
        last_err = float(errs[-1])

        saved_img_path = None
        if self.fig is not None:
            try:
                save_dir = self.image_dir
                os.makedirs(save_dir, exist_ok=True)
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"trajetoria_robo_{self.robot_id}_{timestamp}.png"
                saved_img_path = os.path.join(save_dir, filename)
                self.fig.savefig(saved_img_path, dpi=300, bbox_inches='tight', facecolor=self.fig.get_facecolor())
                latest_path = os.path.join(save_dir, "trajetoria_latest.png")
                self.fig.savefig(latest_path, dpi=300, bbox_inches='tight', facecolor=self.fig.get_facecolor())
            except Exception as exc:
                print(f"[Plotter] Notice: Unable to save trajectory figure: {exc}")

        print('\n' + '=' * 60)
        print('       RELATÓRIO FINAL DE ANÁLISE DE TRAJETÓRIA (mm)       ')
        print('=' * 60)
        print(f'  Robô Analisado        : {self.robot_id}')
        print(f'  Tempo Total Gravado   : {total_time:.2f} s')
        print(f'  Amostras Coletadas    : {len(errs)}')
        print('-' * 60)
        print(f'  Erro Médio (MAE)      : {mae:10.2f} mm  ({mae / 10.0:6.2f} cm)')
        print(f'  RMSE                  : {rmse:10.2f} mm  ({rmse / 10.0:6.2f} cm)')
        print(f'  Desvio Padrão         : {std_err:10.2f} mm  ({std_err / 10.0:6.2f} cm)')
        print(f'  Erro Máximo           : {max_err:10.2f} mm  ({max_err / 10.0:6.2f} cm)')
        print(f'  Erro Mínimo           : {min_err:10.2f} mm  ({min_err / 10.0:6.2f} cm)')
        print(f'  Erro Final            : {last_err:10.2f} mm  ({last_err / 10.0:6.2f} cm)')
        if saved_img_path:
            print(f'  Imagem Salva em       : {saved_img_path}')
        print('=' * 60 + '\n')


def main(args: list[str] | None = None) -> None:
    """Execute the plotting node and manage Matplotlib GUI lifecycle.

    Args:
        args: Command-line arguments forwarded to rclpy initialization.
    """
    rclpy.init(args=args)
    node = TrajectoryPlotterNode()

    # Execute ROS 2 communication in a background thread to maintain GUI responsiveness
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Configure focused Matplotlib canvas with dark aesthetic
    fig, ax = plt.subplots(figsize=(9, 8), facecolor='#0f172a')
    node.fig = fig
    ax.set_facecolor('#1e293b')
    ax.set_title(f'Trajetória de Teste - Robô {node.robot_id} [mm]', color='white', fontsize=13, pad=12)
    ax.set_xlabel('X [mm]', color='white', fontsize=11)
    ax.set_ylabel('Y [mm]', color='white', fontsize=11)
    ax.tick_params(colors='white')
    ax.grid(True, linestyle='--', alpha=0.35, color='gray')
    ax.set_aspect('equal')
    ax.set_xlim(node.min_x, node.max_x)
    ax.set_ylim(node.min_y, node.max_y)

    # Graphical artists for trajectory paths, markers, and heading vectors
    line_plan, = ax.plot(
        [], [], 'c--', linewidth=1.4, marker='.', markersize=4.5,
        markeredgecolor='cyan', label='Rota Desejada (Pontos Discretos)', alpha=0.85, zorder=3
    )
    line_real, = ax.plot(
        [], [], '#facc15', linewidth=1.8, marker='o', markersize=2.5, markevery=6,
        label='Trajetória Real (Amostras 10Hz)', zorder=4
    )
    target_line, = ax.plot([], [], ':', color='#f43f5e', linewidth=1.8, alpha=0.8, label='Perseguição ao Alvo', zorder=5)
    robot_marker, = ax.plot([], [], 'o', color='#38bdf8', markersize=10, label='Robô', zorder=6)
    heading_line, = ax.plot([], [], color='#ef4444', linewidth=2.5, label='Orientação', zorder=7)
    goal_marker, = ax.plot(
        [], [], '*', color='#ff1744', markersize=18,
        markeredgecolor='white', markeredgewidth=1.8, label='Alvo Atual', zorder=8
    )

    ax.legend(loc='upper right', facecolor='#0f172a', edgecolor='gray', labelcolor='white', fontsize=10)

    # Floating HUD displaying real-time trajectory tracking telemetry
    stat_box = ax.text(
        0.03, 0.95, 'Aguardando dados...', transform=ax.transAxes,
        color='white', fontsize=10, verticalalignment='top',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f172a', alpha=0.85, edgecolor='gray')
    )

    def update(frame: Any) -> tuple[Any, ...]:
        """Animation update function executed periodically by FuncAnimation.

        Args:
            frame: Animation frame index supplied by Matplotlib.

        Returns:
            tuple[Any, ...]: Updated visual artists for rendering.
        """
        try:
            with node.lock:
                # Adjust axis limits to focus on reference figure with configured padding
                if node.bounds_set:
                    cur_xlim = ax.get_xlim()
                    cur_ylim = ax.get_ylim()
                    if (abs(cur_xlim[0] - node.min_x) > 1.0 or abs(cur_xlim[1] - node.max_x) > 1.0 or
                            abs(cur_ylim[0] - node.min_y) > 1.0 or abs(cur_ylim[1] - node.max_y) > 1.0):
                        ax.set_xlim(node.min_x, node.max_x)
                        ax.set_ylim(node.min_y, node.max_y)

                # Update planned reference geometry
                if node.plan_x_mm:
                    line_plan.set_data(node.plan_x_mm, node.plan_y_mm)

                # Update recorded robot trajectory
                if node.real_x_mm:
                    line_real.set_data(list(node.real_x_mm), list(node.real_y_mm))

                # Update robot position marker and heading vector (90 mm arrow)
                if node.current_x_mm is not None and node.current_y_mm is not None:
                    robot_marker.set_data([node.current_x_mm], [node.current_y_mm])
                    arrow_len_mm = 90.0
                    hx = node.current_x_mm + arrow_len_mm * math.cos(node.current_yaw)
                    hy = node.current_y_mm + arrow_len_mm * math.sin(node.current_yaw)
                    heading_line.set_data([node.current_x_mm, hx], [node.current_y_mm, hy])

                # Resolve coordinates of active lookahead goal
                gx = node.goal_x_mm
                gy = node.goal_y_mm
                if gx is None and node.plan_x_mm:
                    gx = node.plan_x_mm[0]
                    gy = node.plan_y_mm[0]

                if gx is not None and gy is not None:
                    goal_marker.set_data([gx], [gy])
                    if node.current_x_mm is not None and node.current_y_mm is not None:
                        target_line.set_data([node.current_x_mm, gx], [node.current_y_mm, gy])

                # Update floating numerical display box
                if node.error_history_mm:
                    latest = node.error_history_mm[-1]
                    avg = float(np.mean(node.error_history_mm))
                    goal_str = f'({gx:.0f}, {gy:.0f}) mm' if gx is not None and gy is not None else 'N/A'
                    stat_box.set_text(
                        f'Alvo Atual : {goal_str}\n'
                        f'Erro Atual : {latest:6.1f} mm\n'
                        f'Erro Médio : {avg:6.1f} mm\n'
                        f'Amostras   : {len(node.error_history_mm)}'
                    )
        except Exception:
            import traceback
            traceback.print_exc()

        return line_plan, line_real, target_line, robot_marker, heading_line, goal_marker, stat_box

    def on_close(event: Any) -> None:
        """Handle window closure event by triggering statistical reporting."""
        node.print_error_summary()

    fig.canvas.mpl_connect('close_event', on_close)

    node.anim = FuncAnimation(
        fig, update, interval=node.interval_ms, blit=False, cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()

    # Guarantee summary output if interrupted or closed
    node.print_error_summary()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
