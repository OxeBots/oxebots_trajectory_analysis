"""Unit tests for oxebots_trajectory_analysis module using pytest."""

import math
import numpy as np
import pytest

from oxebots_trajectory_analysis.trajectory_generator_node import (
    normalize_pattern,
    yaw_to_quaternion,
)


def test_normalize_pattern():
    """Verify alias normalization to canonical pattern identifiers."""
    assert normalize_pattern('circulo') == 'circle'
    assert normalize_pattern('CIRCLE') == 'circle'
    assert normalize_pattern('c') == 'circle'

    assert normalize_pattern('quadrado') == 'square'
    assert normalize_pattern('square') == 'square'
    assert normalize_pattern('sq') == 'square'

    assert normalize_pattern('onda') == 'square_wave'
    assert normalize_pattern('wave') == 'square_wave'
    assert normalize_pattern('square_wave') == 'square_wave'
    assert normalize_pattern('sw') == 'square_wave'


def test_yaw_to_quaternion():
    """Verify conversion of planar yaw angles to normalized 3D quaternions."""
    # Yaw = 0 rad -> Identity rotation around Z
    q0 = yaw_to_quaternion(0.0)
    assert q0.x == 0.0
    assert q0.y == 0.0
    assert pytest.approx(q0.z, abs=1e-6) == 0.0
    assert pytest.approx(q0.w, abs=1e-6) == 1.0

    # Yaw = pi rad -> 180 degrees turn around Z
    q_pi = yaw_to_quaternion(math.pi)
    assert pytest.approx(q_pi.z, abs=1e-6) == 1.0
    assert pytest.approx(q_pi.w, abs=1e-6) == 0.0

    # Yaw = pi/2 rad -> 90 degrees turn around Z
    q_pi2 = yaw_to_quaternion(math.pi / 2.0)
    assert pytest.approx(q_pi2.z, abs=1e-6) == math.sin(math.pi / 4.0)
    assert pytest.approx(q_pi2.w, abs=1e-6) == math.cos(math.pi / 4.0)


def test_circle_geometry_discretization():
    """Verify that a circular path has uniform radius and step distance."""
    radius = 0.8  # meters (800 mm)
    num_points = 150
    center_x = 0.0
    center_y = 0.0

    thetas = np.linspace(0.0, 2.0 * math.pi, num_points, endpoint=False)
    points = []
    for th in thetas:
        x = center_x + radius * math.cos(th)
        y = center_y + radius * math.sin(th)
        points.append((x, y))

    assert len(points) == 150

    # Check radius for each point
    for x, y in points:
        r = math.hypot(x - center_x, y - center_y)
        assert pytest.approx(r, abs=1e-5) == radius

    # Check step distance between consecutive points (~33.5 mm)
    expected_step = (2.0 * math.pi * radius) / num_points
    for i in range(num_points):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % num_points]
        step = math.hypot(x2 - x1, y2 - y1)
        assert pytest.approx(step, rel=1e-2) == expected_step


def test_square_geometry_boundaries():
    """Verify that all points in a square path stay strictly within field boundaries."""
    side = 1.4  # meters (1400 mm)
    half = side / 2.0
    num_points = 150

    s_vals = np.linspace(0.0, 4.0 * side, num_points, endpoint=False)
    for s in s_vals:
        if s < side:
            x = -half + s
            y = -half
        elif s < 2.0 * side:
            x = half
            y = -half + (s - side)
        elif s < 3.0 * side:
            x = half - (s - 2.0 * side)
            y = half
        else:
            x = -half
            y = half - (s - 3.0 * side)

        assert -half - 1e-6 <= x <= half + 1e-6
        assert -half - 1e-6 <= y <= half + 1e-6
