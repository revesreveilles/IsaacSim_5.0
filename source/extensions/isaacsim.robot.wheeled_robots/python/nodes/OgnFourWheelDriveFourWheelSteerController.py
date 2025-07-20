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
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.four_wheel_drive_four_wheel_steer_controller import FourWheelDriveFourWheelSteerController
from isaacsim.robot.wheeled_robots.ogn.OgnFourWheelDriveFourWheelSteerControllerDatabase import OgnFourWheelDriveFourWheelSteerControllerDatabase


class OgnFourWheelDriveFourWheelSteerControllerInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_base = 0.0
        self.track_width = 0.0
        self.wheel_radii = np.array([0.1, 0.1, 0.1, 0.1])  
        self.wheel_positions = np.zeros((4, 3)) 
        self.max_wheel_velocity = 1.0e20
        self.max_steering_angle = np.pi/2
        self.max_acceleration = 0.0
        self.max_steering_velocity = 0.0

        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        # 简化控制器创建
        self.controller_handle = FourWheelDriveFourWheelSteerController(
            name="four_wheel_drive_four_wheel_steer_controller",
            wheel_base=self.wheel_base,
            track_width=self.track_width,
            wheel_radii=np.asarray(self.wheel_radii),        
            wheel_positions=np.asarray(self.wheel_positions),
            max_wheel_velocity=self.max_wheel_velocity,
            max_steering_angle=self.max_steering_angle,
            max_acceleration=self.max_acceleration,
            max_steering_velocity=self.max_steering_velocity,
        )
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        return self.controller_handle.forward(command)

    def custom_reset(self):
        self.node.get_attribute("outputs:wheelVelocities").set([0.0, 0.0, 0.0, 0.0])
        self.node.get_attribute("outputs:steeringAngles").set([0.0, 0.0, 0.0, 0.0])


class OgnFourWheelDriveFourWheelSteerController:
    @staticmethod
    def init_instance(node, graph_instance_id):
        state = OgnFourWheelDriveFourWheelSteerControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def internal_state():
        return OgnFourWheelDriveFourWheelSteerControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.wheel_base = db.inputs.wheelBase
                state.track_width = db.inputs.trackWidth
                
                if db.inputs.wheelRadii is not None:
                    state.wheel_radii = np.array(db.inputs.wheelRadii)
                
                if db.inputs.wheelPositions is not None:
                    if len(db.inputs.wheelPositions) == 4:
                        state.wheel_positions = np.array(db.inputs.wheelPositions)
                    else:
                        carb.log_warn(f"[4WD4WS] wheelPositions count {len(db.inputs.wheelPositions)} != 4, using default")
                
                state.max_wheel_velocity = db.inputs.maxWheelVelocity
                state.max_steering_angle = db.inputs.maxSteeringAngle
                state.max_acceleration = db.inputs.maxAcceleration
                state.max_steering_velocity = db.inputs.maxSteeringVelocity

                state.initialize_controller()

            command = [
                db.inputs.chassisLinearVel,   # vectord[3] -> 控制器内转换为np.array
                db.inputs.chassisAngularVel,  # vectord[3] -> 控制器内转换为np.array
                db.inputs.yaw,               # double
                db.inputs.dt                 # double
            ]

            # 执行控制器（核心算法在controller中）
            actions = state.forward(command)

            if actions.joint_velocities is not None:
                db.outputs.wheelVelocities = list(actions.joint_velocities)

            if actions.joint_positions is not None:
                db.outputs.steeringAngles = list(actions.joint_positions)

        except Exception as error:
            carb.log_error(f"4WD4WSController compute error: {error}")
            db.outputs.wheelVelocities = [0.0] * 4
            db.outputs.steeringAngles = [0.0] * 4
            return False

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnFourWheelDriveFourWheelSteerControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
            state.initialized = False
