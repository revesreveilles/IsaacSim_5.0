# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List

import carb
import numpy as np
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

from .path_planning_interface import PathPlanner


class PathPlannerVisualizer:
    """A helper class for quickly visualizing the plans output by a PathPlanner.
    The main utility of this class lies in the compute_plan_as_articulation_actions() function, which returns a sequence of
    ArticulationActions that may be directly sent to the robot Articulation in order to visualize the planned path.

    Args:
        robot_articulation (SingleArticulation): An Articulation object describing a single simulated robot.
        path_planner (PathPlanner):  A PathPlanner object that has been configured to compute plans for the robot
            represented by the robot Articulation.
    """

    def __init__(self, robot_articulation: SingleArticulation, path_planner: PathPlanner) -> None:
        self._robot_articulation = robot_articulation

        self._planner = path_planner

        self._articulation_controller = self._robot_articulation.get_articulation_controller()

        self._active_joints_view = ArticulationSubset(robot_articulation, path_planner.get_active_joints())
        self._watched_joints_view = ArticulationSubset(robot_articulation, path_planner.get_watched_joints())

    def compute_plan_as_articulation_actions(self, max_cspace_dist: float = 0.05) -> List[ArticulationAction]:
        """Compute plan using a PathPlanner and linearly interpolate the result to enforce that the maximum
        distance (l2 norm) between any two points is max_cspace_dist.

        Args:
            max_cspace_dist (float, optional): Maximum distance between adjacent points in the path. Defaults to 0.05.

        Returns:
            List[ArticulationAction]: Linearly interpolated path given as a sequence of ArticulationActions that can be
                passed directly to the robot Articulation.  This may rearrange and augment the plan output by the PathPlanner to
                match the number of DOFs available for control in the robot Articulation.
        """
        active_joint_positions = self._active_joints_view.get_joint_positions()

        if active_joint_positions is None:
            carb.log_error(
                "Attempted to compute a path for an uninitialized robot Articulation. Cannot get joint positions"
            )

        watched_joint_positions = self._watched_joints_view.get_joint_positions()

        path = self._planner.compute_path(active_joint_positions, watched_joint_positions)
        if path is None:
            return []

        interpolated_path = self.interpolate_path(path, max_cspace_dist)

        articulation_actions = [
            self._active_joints_view.make_articulation_action(interpolated_path[i], None)
            for i in range(len(interpolated_path))
        ]

        return articulation_actions

    def interpolate_path(self, path: np.array, max_cspace_dist: float = 0.05) -> np.array:
        """Linearly interpolate a sparse path such that the maximum distance (l2 norm) between any two points is max_cspace_dist

        Args:
            path (np.array): Sparse cspace path with shape (N x num_dofs) where N is number of points in the path
            max_cspace_dist (float, optional): _description_. Defaults to 0.05.

        Returns:
            np.array: Linearly interpolated path with shape (M x num_dofs)
        """
        if path.shape[0] == 0:
            return path

        interpolated_path = []
        for i in range(path.shape[0] - 1):
            n_pts = int(np.ceil(np.amax(abs(path[i + 1] - path[i])) / max_cspace_dist))
            interpolated_path.append(np.array(np.linspace(path[i], path[i + 1], num=n_pts, endpoint=False)))
        interpolated_path.append(path[np.newaxis, -1, :])

        interpolated_path = np.concatenate(interpolated_path)
        return interpolated_path

    def get_active_joints_subset(self) -> ArticulationSubset:
        """Get view into active joints

        Returns:
            ArticulationSubset: Returns robot states for active joints in an order compatible with the PathPlanner
        """
        return self._active_joints_view

    def get_watched_joints_subset(self) -> ArticulationSubset:
        """Get view into watched joints

        Returns:
            ArticulationSubset: Returns robot states for watched joints in an order compatible with the PathPlanner
        """
        return self._watched_joints_view

    def get_robot_articulation(self) -> SingleArticulation:
        """Get the robot Articulation

        Returns:
            SingleArticulation: Articulation object describing the robot.
        """
        return self._robot_articulation

    def get_path_planner(self) -> PathPlanner:
        """Get the PathPlanner that is being used to generate paths

        Returns:
            PathPlanner: An instance of the PathPlanner interface for generating sparse paths to a target pose
        """
        return self._planner
