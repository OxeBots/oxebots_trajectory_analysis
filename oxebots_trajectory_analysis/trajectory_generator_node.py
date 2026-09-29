#!/usr/bin/env python3
"""Analytical trajectory generation and tracking evaluation node for RoboCup SSL.

This module provides the TrajectoryGeneratorNode, which synthesizes spatially
discretized geometric reference paths (circle, square, square wave) within the
RoboCup Small Size League (SSL) coordinate system. The node tracks robot progress
using global vision telemetry and streams a sliding window of upcoming waypoints
to the low-level path follower.

Academic references and standards:
    - Pure Pursuit path following with feedforward curvature compensation.
    - PEP 8 (Style Guide for Python Code) and PEP 257 (Docstring Conventions).
"""

from __future__ import annotations

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from oxebots_interfaces.msg import GameData, RobotGoal


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a planar yaw angle into a geometry_msgs Quaternion.

    Args:
        yaw: Planar rotation angle around the Z-axis in radians.

    Returns:
        Quaternion: Normalized orientation quaternion representation.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_pattern(raw: str) -> str:
    """Normalize user-supplied pattern string aliases into canonical identifiers.

    Args:
        raw: Raw pattern name or common alias (e.g., 'circulo', 'wave', 'quadrado').

    Returns:
        str: Canonical pattern string ('circle', 'square', or 'square_wave').
    """
    p = str(raw).strip().lower()
    if p in ('circle', 'circulo', 'c'):
        return 'circle'
    if p in ('square', 'quadrado', 'sq', 'q'):
        return 'square'
    if p in ('square_wave', 'wave', 'onda', 'onda_quadrada', 'onda-quadrada', 'squarewave', 'sw'):
        return 'square_wave'
    return p


class TrajectoryGeneratorNode(Node):
    """ROS 2 Node for analytical trajectory synthesis and progress tracking.

    Attributes:
        robot_id (int): Designated robot identifier.
        pattern (str): Active geometric pattern under evaluation.
        rate_hz (float): Publishing and control loop execution frequency in Hz.
        frame_id (str): Global reference frame identifier (default: 'map').
        total_points (int): Total number of discrete waypoints along the geometric path.
        path_window_size (int): Size of the sliding waypoint window sent to the follower.
        full_points (list[tuple[float, float, float]]): Discretized (x, y, yaw) coordinates.
        current_idx (int): Current active reference waypoint index.
    """

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscriptions, and state variables."""
        super().__init__('trajectory_generator_node')

        # General operational parameters
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('pattern', 'circle')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('total_points', 150)
        self.declare_parameter('path_window_size', 15)
        self.declare_parameter('lookahead_steps', 2)
        self.declare_parameter('face_center', True)

        # Geometric parameters for circular trajectories
        self.declare_parameter('circle_radius', 0.8)
        self.declare_parameter('circle_center_x', 0.0)
        self.declare_parameter('circle_center_y', 0.0)

        # Geometric parameters for square trajectories
        self.declare_parameter('square_side', 1.4)
        self.declare_parameter('square_center_x', 0.0)
        self.declare_parameter('square_center_y', 0.0)

        # Geometric parameters for square-wave trajectories
        self.declare_parameter('wave_amplitude', 0.5)
        self.declare_parameter('wave_step_length', 0.6)
        self.declare_parameter('wave_num_cycles', 2)
        self.declare_parameter('wave_center_x', 0.0)
        self.declare_parameter('wave_center_y', 0.0)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        raw_pattern = self.get_parameter('pattern').get_parameter_value().string_value
        self.pattern = normalize_pattern(raw_pattern)
        self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.total_points = int(self.get_parameter('total_points').value)
        self.path_window_size = int(self.get_parameter('path_window_size').value)

        # State telemetry from global vision system
        self.robot_x: float | None = None
        self.robot_y: float | None = None
        self.robot_yaw: float = 0.0

        # Discretized reference path and progress indicators
        self.full_points = self.build_static_points()
        self.current_idx: int = 0
        self.initialized_start: bool = False
        self.wave_direction: int = 1

        robot_prefix = f'/robot_{self.robot_id}'
        self.path_pub = self.create_publisher(Path, f'{robot_prefix}/path', 10)
        self.viz_path_pub = self.create_publisher(Path, f'{robot_prefix}/path_viz', 10)
        self.goal_pub = self.create_publisher(RobotGoal, f'{robot_prefix}/target_goal', 10)
        self.global_goal_pub = self.create_publisher(RobotGoal, '/robot_goal', 10)

        self.game_data_sub = self.create_subscription(
            GameData, '/game_data', self.game_data_cb, 10
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Trajectory Generator started for robot {self.robot_id} with pattern: "{self.pattern}" '
            f'({len(self.full_points)} total discrete points, window: {self.path_window_size} points).'
        )

    def game_data_cb(self, msg: GameData) -> None:
        """Process incoming vision data to track the robot's real-time pose.

        Args:
            msg: GameData message containing global SSL-Vision positions (in mm).
        """
        for ally in msg.robots.allies:
            if ally.id == self.robot_id:
                # Convert vision coordinates from millimeters to meters
                self.robot_x = float(ally.x) / 1000.0
                self.robot_y = float(ally.y) / 1000.0
                self.robot_yaw = float(ally.orientation)
                break

    def build_static_points(self) -> list[tuple[float, float, float]]:
        """Construct the static discretized reference trajectory.

        Computes the coordinates (x, y) in meters and target heading (yaw) in radians
        for all discrete nodes along the selected geometric contour.

        Returns:
            list[tuple[float, float, float]]: Ordered list of (x, y, yaw) coordinates.
        """
        pattern = self.pattern
        num_points = self.total_points
        face_center = bool(self.get_parameter('face_center').value)

        if pattern == 'circle':
            radius = float(self.get_parameter('circle_radius').value)
            center_x = float(self.get_parameter('circle_center_x').value)
            center_y = float(self.get_parameter('circle_center_y').value)

            points: list[tuple[float, float, float]] = []
            thetas = np.linspace(0.0, 2.0 * math.pi, num_points, endpoint=False)
            for th in thetas:
                x = center_x + radius * math.cos(th)
                y = center_y + radius * math.sin(th)
                yaw = math.atan2(center_y - y, center_x - x) if face_center else th + math.pi / 2.0
                points.append((float(x), float(y), float(yaw)))
            return points

        elif pattern == 'square':
            side = float(self.get_parameter('square_side').value)
            center_x = float(self.get_parameter('square_center_x').value)
            center_y = float(self.get_parameter('square_center_y').value)
            half = side / 2.0

            # Uniform arc-length discretization along the 4 perimeter edges
            s_vals = np.linspace(0.0, 4.0 * side, num_points, endpoint=False)
            points: list[tuple[float, float, float]] = []
            for s in s_vals:
                if s < side:
                    # Bottom side: moving from left (-half) to right (+half)
                    x = (center_x - half) + s
                    y = center_y - half
                    tangent_yaw = 0.0
                elif s < 2.0 * side:
                    # Right side: moving from bottom (-half) to top (+half)
                    x = center_x + half
                    y = (center_y - half) + (s - side)
                    tangent_yaw = math.pi / 2.0
                elif s < 3.0 * side:
                    # Top side: moving from right (+half) to left (-half)
                    x = (center_x + half) - (s - 2.0 * side)
                    y = center_y + half
                    tangent_yaw = math.pi
                else:
                    # Left side: moving from top (+half) to bottom (-half)
                    x = center_x - half
                    y = (center_y + half) - (s - 3.0 * side)
                    tangent_yaw = -math.pi / 2.0

                yaw = math.atan2(center_y - y, center_x - x) if face_center else tangent_yaw
                points.append((float(x), float(y), float(yaw)))

            return points

        elif pattern == 'square_wave':
            amp = float(self.get_parameter('wave_amplitude').value)
            step_len = float(self.get_parameter('wave_step_length').value)
            cycles = int(self.get_parameter('wave_num_cycles').value)
            center_x = float(self.get_parameter('wave_center_x').value)
            center_y = float(self.get_parameter('wave_center_y').value)

            total_length = cycles * 2 * step_len
            start_x = center_x - total_length / 2.0

            key_points: list[tuple[float, float]] = []
            curr_x = start_x
            curr_y = center_y - amp
            key_points.append((curr_x, curr_y))

            for _ in range(cycles):
                curr_y = center_y + amp
                key_points.append((curr_x, curr_y))
                curr_x += step_len
                key_points.append((curr_x, curr_y))
                curr_y = center_y - amp
                key_points.append((curr_x, curr_y))
                curr_x += step_len
                key_points.append((curr_x, curr_y))

            total_segments = len(key_points) - 1
            pts_per_seg = max(1, num_points // total_segments)
            points = []
            for i in range(total_segments):
                x0, y0 = key_points[i]
                x1, y1 = key_points[i + 1]
                tangent_yaw = math.atan2(y1 - y0, x1 - x0)
                xs = np.linspace(x0, x1, pts_per_seg, endpoint=False)
                ys = np.linspace(y0, y1, pts_per_seg, endpoint=False)
                for x, y in zip(xs, ys):
                    yaw = math.atan2(center_y - y, center_x - x) if face_center else tangent_yaw
                    points.append((float(x), float(y), float(yaw)))

            while len(points) < num_points:
                last_pt = key_points[-1]
                yaw = math.atan2(center_y - last_pt[1], center_x - last_pt[0]) if face_center else float(points[-1][2])
                points.append((float(last_pt[0]), float(last_pt[1]), float(yaw)))
            return points[:num_points]

        self.get_logger().warn(f'Unrecognized pattern: "{pattern}". Defaulting to "square_wave".')
        return self.build_static_points_fallback()

    def build_static_points_fallback(self) -> list[tuple[float, float, float]]:
        """Synthesize default fallback points when an unknown pattern identifier is requested.

        Returns:
            list[tuple[float, float, float]]: Standard square wave fallback points.
        """
        self.pattern = 'square_wave'
        return self.build_static_points()

    def update_sequential_tracker(self) -> None:
        """Update active trajectory index based on real-time robot localization.

        Applies direct angular projection for circular geometry and bounded forward
        search for segmented figures, ensuring monotonically progressive tracking.
        """
        if self.robot_x is None or self.robot_y is None or not self.full_points:
            return

        num_points = len(self.full_points)

        # Circular trajectory: direct angular projection eliminates index lag/lead
        if self.pattern == 'circle':
            center_x = float(self.get_parameter('circle_center_x').value)
            center_y = float(self.get_parameter('circle_center_y').value)
            theta = math.atan2(self.robot_y - center_y, self.robot_x - center_x)
            if theta < 0.0:
                theta += 2.0 * math.pi
            self.current_idx = int(round((theta / (2.0 * math.pi)) * num_points)) % num_points
            self.initialized_start = True
            return

        # Linear segments (Square / Square Wave): initial synchronization to nearest waypoint
        if not self.initialized_start:
            if self.pattern == 'square_wave':
                self.current_idx = 0
            else:
                dists = [
                    math.hypot(pt[0] - self.robot_x, pt[1] - self.robot_y)
                    for pt in self.full_points
                ]
                self.current_idx = int(np.argmin(dists))
            self.initialized_start = True
            return

        best_idx = self.current_idx
        best_dist = math.hypot(
            self.full_points[self.current_idx][0] - self.robot_x,
            self.full_points[self.current_idx][1] - self.robot_y
        )

        # Invert direction upon reaching boundary limits in non-closed contours
        if self.pattern == 'square_wave':
            if self.current_idx >= num_points - 5 and self.wave_direction == 1:
                self.wave_direction = -1
            elif self.current_idx <= 5 and self.wave_direction == -1:
                self.wave_direction = 1

        # Advance along the route only when the robot is within proximity threshold (< 250 mm)
        if best_dist < 0.25:
            search_range = max(5, num_points // 15)
            for step in range(1, search_range + 1):
                if self.pattern == 'square_wave':
                    test_idx = self.current_idx + self.wave_direction * step
                    if test_idx >= num_points or test_idx < 0:
                        continue
                else:
                    test_idx = (self.current_idx + step) % num_points

                pt = self.full_points[test_idx]
                d = math.hypot(pt[0] - self.robot_x, pt[1] - self.robot_y)
                if d < best_dist:
                    best_dist = d
                    best_idx = test_idx
            self.current_idx = best_idx

    def timer_callback(self) -> None:
        """Periodic control callback executed at publish_rate_hz.

        Updates the spatial tracker and publishes:
            1. The active sliding window of waypoints to /robot_<id>/path.
            2. The complete static reference path to /robot_<id>/path_viz.
            3. The active lookahead goal to /robot_<id>/target_goal.
        """
        if not self.full_points:
            return

        self.update_sequential_tracker()

        num_points = len(self.full_points)
        now = self.get_clock().now().to_msg()

        # Compute radial curvature compensation for circular trajectories
        radial_error = 0.0
        if self.pattern == 'circle' and self.robot_x is not None and self.robot_y is not None:
            center_x = float(self.get_parameter('circle_center_x').value)
            center_y = float(self.get_parameter('circle_center_y').value)
            r_nominal = float(self.get_parameter('circle_radius').value)
            r_actual = math.hypot(self.robot_x - center_x, self.robot_y - center_y)
            radial_error = float(np.clip(r_nominal - r_actual, -0.20, 0.20))

        # Active sliding window path published to the low-level PathFollower
        follower_path = Path()
        follower_path.header.stamp = now
        follower_path.header.frame_id = self.frame_id

        num_window_points = min(self.path_window_size, num_points)
        for i in range(num_window_points):
            if self.pattern == 'square_wave':
                idx = max(0, min(num_points - 1, self.current_idx + self.wave_direction * i))
            else:
                idx = (self.current_idx + i) % num_points
            x, y, yaw = self.full_points[idx]

            # Pure Pursuit chord error feedforward and proportional radial feedback
            if self.pattern == 'circle':
                weight = max(0.0, 1.0 - (i / float(num_window_points)))
                center_x = float(self.get_parameter('circle_center_x').value)
                center_y = float(self.get_parameter('circle_center_y').value)
                r_nominal = float(self.get_parameter('circle_radius').value)
                r_corr = r_nominal + weight * (0.0015 + 1.2 * radial_error)
                theta = (idx / num_points) * 2.0 * math.pi
                x = center_x + r_corr * math.cos(theta)
                y = center_y + r_corr * math.sin(theta)

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            follower_path.poses.append(pose)

        self.path_pub.publish(follower_path)

        # Complete reference trajectory (full points) for visualizers (Matplotlib / RViz)
        viz_path = Path()
        viz_path.header.stamp = now
        viz_path.header.frame_id = self.frame_id
        for x, y, yaw in self.full_points:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            viz_path.poses.append(pose)

        self.viz_path_pub.publish(viz_path)

        #Active lookahead target pose for reference visualization
        lookahead_steps = int(self.get_parameter('lookahead_steps').value)
        if self.pattern == 'square_wave':
            goal_idx = max(0, min(num_points - 1, self.current_idx + self.wave_direction * lookahead_steps))
        else:
            goal_idx = (self.current_idx + lookahead_steps) % num_points

        target_x, target_y, target_yaw = self.full_points[goal_idx]

        goal_msg = RobotGoal()
        goal_msg.robot_id = self.robot_id
        goal_msg.planner_type = RobotGoal.PLANNER_STRAIGHT_LINE
        goal_msg.pose.header.stamp = now
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(target_yaw)
        self.goal_pub.publish(goal_msg)
        self.global_goal_pub.publish(goal_msg)


def main(args: list[str] | None = None) -> None:
    """Execute node lifecycle within the ROS 2 executor.

    Args:
        args: Command-line arguments forwarded to rclpy initialization.
    """
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
