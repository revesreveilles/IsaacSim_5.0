#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Test script for 4WD4WS Controller
This script demonstrates the basic functionality of the controller.
"""

import numpy as np
from isaacsim.robot.wheeled_robots.controllers.four_wheel_drive_four_wheel_steer_controller import FourWheelDriveFourWheelSteerController


def test_4wd4ws_controller():
    """Test the 4WD4WS controller with various motion scenarios."""
    
    # Robot parameters (example values)
    wheel_base = 2.0  # meters
    track_width = 1.5  # meters  
    wheel_radius = 0.3  # meters (average)
    max_wheel_velocity = 10.0  # rad/s
    max_steering_angle = np.pi/3  # 60 degrees
    
    # Individual wheel distances from center (optional)
    half_base = wheel_base / 2.0
    half_track = track_width / 2.0
    wheel_distances = np.array([
        np.sqrt(half_base**2 + half_track**2),  # Front left
        np.sqrt(half_base**2 + half_track**2),  # Front right
        np.sqrt(half_base**2 + half_track**2),  # Back left
        np.sqrt(half_base**2 + half_track**2)   # Back right
    ])
    
    # Individual wheel radii (can be different for each wheel)
    wheel_radii = np.array([0.3, 0.3, 0.32, 0.32])  # Slightly larger rear wheels
    
    # Create controller
    controller = FourWheelDriveFourWheelSteerController(
        name="test_4wd4ws",
        wheel_base=wheel_base,
        track_width=track_width,
        wheel_radius=wheel_radius,
        max_wheel_velocity=max_wheel_velocity,
        max_steering_angle=max_steering_angle,
        wheel_distances=wheel_distances,
        wheel_radii=wheel_radii
    )
    
    print("4WD4WS Controller Test")
    print("=" * 50)
    print(f"Robot parameters:")
    print(f"  Wheel base: {wheel_base} m")
    print(f"  Track width: {track_width} m") 
    print(f"  Average wheel radius: {wheel_radius} m")
    print(f"  Individual wheel distances: {wheel_distances}")
    print(f"  Individual wheel radii: {wheel_radii}")
    print(f"  Max wheel velocity: {max_wheel_velocity} rad/s")
    print(f"  Max steering angle: {max_steering_angle:.2f} rad ({np.degrees(max_steering_angle):.1f}°)")
    print()
    
    # Test scenarios
    test_cases = [
        # [linear_velocity, angular_velocity, yaw, dt, description]
        [1.0, 0.0, 0.0, 0.1, "Forward motion"],
        [0.0, 1.0, 0.0, 0.1, "Pure rotation"],
        [1.0, 0.5, 0.0, 0.1, "Forward with left turn"],
        [1.0, -0.5, 0.0, 0.1, "Forward with right turn"],
        [-1.0, 0.0, 0.0, 0.1, "Backward motion"],
        [0.0, 0.0, 0.0, 0.1, "Stationary"],
    ]
    
    for i, (linear_vel, angular_vel, yaw, dt, description) in enumerate(test_cases, 1):
        print(f"Test {i}: {description}")
        print(f"  Command: linear_vel={linear_vel:.1f} m/s, angular_vel={angular_vel:.1f} rad/s")
        
        command = np.array([linear_vel, angular_vel, yaw, dt])
        action = controller.forward(command)
        
        if action.joint_velocities is not None:
            wheel_vels = action.joint_velocities[:4]
            print(f"  Wheel velocities [FL, FR, BL, BR]: {[f'{v:.2f}' for v in wheel_vels]} rad/s")
        
        if action.joint_positions is not None:
            steer_angles = action.joint_positions[4:]
            steer_degrees = np.degrees(steer_angles)
            print(f"  Steering angles [FL, FR, BL, BR]: {[f'{a:.1f}°' for a in steer_degrees]}")
        
        print()
    
    print("Test completed successfully!")


if __name__ == "__main__":
    test_4wd4ws_controller()
