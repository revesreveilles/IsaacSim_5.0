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
from typing import Optional

import carb
import isaacsim.core.utils.stage as stage_utils
import numpy as np
import omni.kit.app
from isaacsim.core.api.materials.visual_material import VisualMaterial
from pxr import Gf, Sdf, UsdShade


class PreviewSurface(VisualMaterial):
    """[summary]

    Args:
        prim_path (str): [description]
        name (str, optional): [description]. Defaults to "preview_surface".
        shader (Optional[UsdShade.Shader], optional): [description]. Defaults to None.
        color (Optional[np.ndarray], optional): [description]. Defaults to None.
        roughness (Optional[float], optional): [description]. Defaults to None.
        metallic (Optional[float], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "preview_surface",
        shader: Optional[UsdShade.Shader] = None,
        color: Optional[np.ndarray] = None,
        roughness: Optional[float] = None,
        metallic: Optional[float] = None,
    ) -> None:
        stage = stage_utils.get_current_stage()
        if stage.GetPrimAtPath(prim_path).IsValid():
            carb.log_info("Material Prim already defined at path: {}".format(prim_path))
            material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            material = UsdShade.Material.Define(stage, prim_path)

        if shader is None:
            if stage.GetPrimAtPath(f"{prim_path}/shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/shader"))
            elif stage.GetPrimAtPath(f"{prim_path}/Shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/Shader"))
            else:
                shader = UsdShade.Shader.Define(stage, f"{prim_path}/shader")
        VisualMaterial.__init__(
            self,
            prim_path=prim_path,
            prim=stage.GetPrimAtPath(prim_path),
            shaders_list=[shader],
            material=material,
            name=name,
        )
        shader.CreateIdAttr("UsdPreviewSurface")
        if color is not None:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        if roughness is not None:
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        if metallic is not None:
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return

    def set_color(self, color: np.ndarray) -> None:
        """[summary]

        Args:
            color (np.ndarray): [description]
        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            self.shaders_list[0].CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        else:
            self.shaders_list[0].GetInput("diffuseColor").Set(Gf.Vec3f(*color.tolist()))
        return

    def get_color(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            carb.log_warn("A color attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("diffuseColor").Get())

    def set_roughness(self, roughness: float) -> None:
        """[summary]

        Args:
            roughness (float): [description]
        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            self.shaders_list[0].CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        else:
            self.shaders_list[0].GetInput("roughness").Set(roughness)
        return

    def get_roughness(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            carb.log_warn("A roughness attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("roughness").Get()

    def set_metallic(self, metallic: float) -> None:
        """[summary]

        Args:
            metallic (float): [description]
        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            self.shaders_list[0].CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        else:
            self.shaders_list[0].GetInput("metallic").Set(metallic)
        return

    def get_metallic(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            carb.log_warn("A metallic attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("metallic").Get()
