# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import carb
import numpy as np
from typing import Optional

# Import packages.
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_rot_matrix,euler_angles_to_quat,euler_to_rot_matrix

class FourWheelDriveFourWheelSteerController(BaseController):
    """
    4WD4WS Controller
    
    Implementation principle:
    1. Calculate wheel velocities for each drive wheel based on chassis linear and angular velocities
    2. Determine steering wheel angles based on wheel velocity directions
    
    Args:
        name (str): Controller name
        wheel_base (float): Wheelbase (front-rear wheel distance) m
        track_width (float): Track width (left-right wheel distance) m  
        wheel_radii (np.ndarray): Array of four wheel radii [front_left, front_right, rear_left, rear_right] m
        wheel_positions (Optional[np.ndarray]): Array of four wheel positions [4x3] [[x,y,z], ...] m, optional, default calculated from wheelbase/trackwidth
        max_wheel_velocity (float): Maximum wheel velocity rad/s
        max_steering_angle (float): Maximum steering angle rad
        max_acceleration (float): Maximum acceleration m/s^2
        max_steering_velocity (float): Maximum steering velocity rad/s
    """

    def __init__(
        self,
        name: str,
        wheel_base: float,
        track_width: float,
        wheel_radii: np.ndarray,                        
        wheel_positions: Optional[np.ndarray] = None,   
        max_wheel_velocity: float = 0.0,
        max_steering_angle: float = np.pi, 
        max_acceleration: float = 0.0,
        max_steering_velocity: float = 0.0,
    ):
        BaseController.__init__(self, name=name)
        
        self._wheel_base = wheel_base
        self._track_width = track_width
        self._half_wheel_base = wheel_base / 2.0
        self._half_track_width = track_width / 2.0
        
        # Wheel radii array processing
        self._wheel_radii = np.array(wheel_radii)
        self._wheel_radius = np.mean(self._wheel_radii)  # Average radius for backward compatibility
        
        # Wheel position processing
        if wheel_positions is not None:
            self._wheel_positions = np.array(wheel_positions)
        else:
            # Use default positions: based on wheel_base and track_width
            self._wheel_positions = np.array([
                [self._half_wheel_base, self._half_track_width, 0.0],    # front left
                [self._half_wheel_base, -self._half_track_width, 0.0],   # front right  
                [-self._half_wheel_base, self._half_track_width, 0.0],   # rear left
                [-self._half_wheel_base, -self._half_track_width, 0.0]   # rear right
            ])
        
        # Control limits
        self._max_wheel_velocity = max_wheel_velocity
        self._max_steering_angle = max_steering_angle
        self._max_acceleration = max_acceleration
        self._max_steering_velocity = max_steering_velocity
        
        # State variables
        self._previous_wheel_velocities = np.zeros(4)
        self._previous_steering_angles = np.zeros(4)
        
        # Current wheel state variables 
        self.wheel_rotation_velocity_FL = 0.0
        self.wheel_rotation_velocity_FR = 0.0
        self.wheel_rotation_velocity_BL = 0.0
        self.wheel_rotation_velocity_BR = 0.0
        
        self.steering_angle_FL = 0.0
        self.steering_angle_FR = 0.0
        self.steering_angle_BL = 0.0
        self.steering_angle_BR = 0.0       

    def forward(self, command) -> ArticulationAction:
        """
        4WD4Ws Controller forward method
        
        Args:
            command: [chassis_linear_vel(vector3), chassis_angular_vel(vector3), yaw(double), dt(s)]

        Returns:
            ArticulationAction: contain wheel velocities and steering angles
        """
        if len(command) < 4:
            carb.log_error("4WD4WS Controller requires command [chassis_linear_vel, chassis_angular_vel, yaw, dt]")
            return ArticulationAction()
        
        # if max wheel velocity is equal to zero, interpret max value as infinity
        if self._max_wheel_velocity == 0.0:
            self._max_wheel_velocity = np.inf

        # if max steering angle is equal to zero, interpret max value as infinity
        if self._max_steering_angle == 0.0:
            self._max_steering_angle = np.inf

        # if max acceleration is equal to zero, interpret max value as infinity
        if self._max_acceleration == 0.0:
            self._max_acceleration = np.inf

        # if max steering velocity is equal to zero, interpret max value as infinity
        if self._max_steering_velocity == 0.0:
            self._max_steering_velocity = np.inf

        # compute max linear velocity
        self.max_linear_velocity = np.fabs(self._max_wheel_velocity * self._wheel_radius)

        # Extract vector parameters 
        if isinstance(command[0], (list, tuple, np.ndarray)):
            chassis_linear_vel = np.array(command[0])
        else:
            chassis_linear_vel = np.array([command[0], 0.0, 0.0])
            
        if isinstance(command[1], (list, tuple, np.ndarray)):
            chassis_angular_vel = np.array(command[1])
        else:
            chassis_angular_vel = np.array([0.0, 0.0, command[1]])
            
        yaw = float(command[2])
        dt = float(command[3])

        # apply input command limits 
        linear_speed = np.linalg.norm(chassis_linear_vel[:2])
        if linear_speed > self.max_linear_velocity:
            scale = self.max_linear_velocity / linear_speed
            chassis_linear_vel[:2] *= scale

        chassis_angular_vel[2] = np.clip(
            chassis_angular_vel[2], -self._max_steering_velocity, self._max_steering_velocity
        )

        # ensure dt is always positive
        dt = np.fabs(dt)

        # check dt validity
        if dt == 0.0 and self._max_acceleration != np.inf:
            carb.log_warn(f"invalid dt {dt}, cannot check for acceleration limits, skipping current step")
            return ArticulationAction()

        # Entrance for 4WD4WS kinematics computation
        wheel_velocities, steering_angles = self._vector_kinematics_motion(
            chassis_linear_vel, chassis_angular_vel, yaw)
            
        # update current state variables 
        self.wheel_rotation_velocity_FL = wheel_velocities[0]
        self.wheel_rotation_velocity_FR = wheel_velocities[1]
        self.wheel_rotation_velocity_BL = wheel_velocities[2] 
        self.wheel_rotation_velocity_BR = wheel_velocities[3]
        
        self.steering_angle_FL = steering_angles[0]
        self.steering_angle_FR = steering_angles[1]
        self.steering_angle_BL = steering_angles[2]
        self.steering_angle_BR = steering_angles[3]
        
        # print(f"Before Apply Limits: wheels={[self.wheel_rotation_velocity_FL, self.wheel_rotation_velocity_FR, self.wheel_rotation_velocity_BL, self.wheel_rotation_velocity_BR]}")
        # print(f"Before Apply Limits: steers={[np.degrees(self.steering_angle_FL), np.degrees(self.steering_angle_FR), np.degrees(self.steering_angle_BL), np.degrees(self.steering_angle_BR)]} degrees)")

        # apply wheel velocity limits 
        self.wheel_rotation_velocity_FL = np.clip(
            self.wheel_rotation_velocity_FL, -self._max_wheel_velocity, self._max_wheel_velocity
        )
        self.wheel_rotation_velocity_FR = np.clip(
            self.wheel_rotation_velocity_FR, -self._max_wheel_velocity, self._max_wheel_velocity
        )
        self.wheel_rotation_velocity_BL = np.clip(
            self.wheel_rotation_velocity_BL, -self._max_wheel_velocity, self._max_wheel_velocity
        )
        self.wheel_rotation_velocity_BR = np.clip(
            self.wheel_rotation_velocity_BR, -self._max_wheel_velocity, self._max_wheel_velocity
        )

        # apply steering angle limits 
        self.steering_angle_FL = np.clip(
            self.steering_angle_FL, -self._max_steering_angle, self._max_steering_angle
        )
        self.steering_angle_FR = np.clip(
            self.steering_angle_FR, -self._max_steering_angle, self._max_steering_angle
        )
        self.steering_angle_BL = np.clip(
            self.steering_angle_BL, -self._max_steering_angle, self._max_steering_angle
        )
        self.steering_angle_BR = np.clip(
            self.steering_angle_BR, -self._max_steering_angle, self._max_steering_angle
        )

        # update historical state
        self._previous_wheel_velocities = np.array([
            self.wheel_rotation_velocity_FL, self.wheel_rotation_velocity_FR,
            self.wheel_rotation_velocity_BL, self.wheel_rotation_velocity_BR
        ])
        self._previous_steering_angles = np.array([
            self.steering_angle_FL, self.steering_angle_FR,
            self.steering_angle_BL, self.steering_angle_BR
        ])
        
        # print(f"限制后最终状态: wheels={[self.wheel_rotation_velocity_FL, self.wheel_rotation_velocity_FR, self.wheel_rotation_velocity_BL, self.wheel_rotation_velocity_BR]}")
        # print(f"限制后最终状态: steers={[np.degrees(self.steering_angle_FL), np.degrees(self.steering_angle_FR), np.degrees(self.steering_angle_BL), np.degrees(self.steering_angle_BR)]} degrees")
        # print(f"最大限制值: max_wheel_vel={self._max_wheel_velocity}, max_steer_angle={np.degrees(self._max_steering_angle):.2f}°")
        
        return ArticulationAction(
            joint_velocities=(
                self.wheel_rotation_velocity_FL,
                self.wheel_rotation_velocity_FR,
                self.wheel_rotation_velocity_BL,
                self.wheel_rotation_velocity_BR,
            ),
            joint_positions=(
                self.steering_angle_FL,
                self.steering_angle_FR,
                self.steering_angle_BL,
                self.steering_angle_BR,
            ),
        )
    
    def _vector_kinematics_motion(self, chassis_linear_vel: np.ndarray, chassis_angular_vel: np.ndarray, yaw: float):
        """
        4WD4WS kinematics calculation 
        
        Args:
            chassis_linear_vel (np.ndarray): Desired linear velocity vector [x, y, z] m/s
            chassis_angular_vel (np.ndarray): Desired angular velocity vector [x, y, z] rad/s  
            yaw (float): Current heading angle rad (for coordinate transformation)
            
        Returns:
            tuple: (wheel_velocities, steering_angles)
        """
        v = np.array([chassis_linear_vel[0], chassis_linear_vel[1], 0.0])  # Eigen::Vector3d
        w = np.array([0.0, 0.0, chassis_angular_vel[2]])                  # Eigen::Vector3d
        
        # Use euler_to_rot_matrix to directly generate inverse transformation matrix R(-yaw) 
        rotation_matrix = euler_to_rot_matrix(np.array([0, 0, -yaw]))
        v = rotation_matrix @ v  
        # print(f"rotation_matrix={rotation_matrix}")
        # print(f"yaw={yaw:.4f}rad ({np.degrees(yaw):.1f}°), using inverse transform R(-yaw)")
        # print(f"v_transformed={v}")
        
        wheel_positions = self._wheel_positions
        
        # print(f"Wheel positions (using {'custom' if hasattr(self, '_custom_positions') else 'default'} positions):")
        # wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
        # for i, (name, pos) in enumerate(zip(wheel_names, wheel_positions)):
        #     print(f"  {name}[{i}]: r = {pos}")
        
        wheel_velocities = np.zeros(4)
        steering_angles = np.zeros(4)
        
        # for each wheel, calculate the velocity and steering angle 
        for i in range(4):
            r = wheel_positions[i]  
            w_cross_r = np.cross(w, r)  # 3D cross product
            vel = v + w_cross_r
            
            vel_norm = np.linalg.norm(vel)
            if vel_norm < 1e-6:
                # when velocity is close to zero
                wheel_velocities[i] = 0.0
                steering_angles[i] = 0.0
            else:
                # Always use positive wheel velocity, direction is handled by steering angle
                wheel_velocities[i] = vel_norm
                
                # Calculate steering angle directly from velocity vector
                # arctan2 already handles all quadrants correctly, including backward motion
                steering_angles[i] = -np.arctan2(vel[1], vel[0]) # Negative sign to adapt to Isaac Sim coordinate system (or USD file)

        return wheel_velocities, steering_angles

    def reset(self):
        """Reset controller state"""
        BaseController.reset(self)
        self._previous_wheel_velocities = np.zeros(4)
        self._previous_steering_angles = np.zeros(4)
        
        # Reset current state variables
        self.wheel_rotation_velocity_FL = 0.0
        self.wheel_rotation_velocity_FR = 0.0
        self.wheel_rotation_velocity_BL = 0.0
        self.wheel_rotation_velocity_BR = 0.0
        
        self.steering_angle_FL = 0.0
        self.steering_angle_FR = 0.0
        self.steering_angle_BL = 0.0
        self.steering_angle_BR = 0.0
        
        print(f"4WD4WS controller {self._name} has been reset")

    @property
    def wheel_base(self) -> float:
        return self._wheel_base
        
    @property 
    def track_width(self) -> float:
        return self._track_width
        
    @property
    def wheel_radius(self) -> float:
        return self._wheel_radius
