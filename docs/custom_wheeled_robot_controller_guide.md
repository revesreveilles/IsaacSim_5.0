# 创建自定义轮式机器人控制器指南

本文档详细介绍了如何在Isaac Sim 5.0的`isaacsim.robot.wheeled_robots`扩展中创建自定义轮式机器人控制器。

## 目录

1. [概述](#概述)
2. [文件结构](#文件结构)
3. [步骤1：创建控制器逻辑](#步骤1创建控制器逻辑)
4. [步骤2：创建OGN节点定义](#步骤2创建ogn节点定义)
5. [步骤3：创建OGN节点实现](#步骤3创建ogn节点实现)
6. [步骤4：构建项目](#步骤4构建项目)
7. [控制器模板](#控制器模板)

## 概述

Isaac Sim使用OmniGraph(OGN)系统来创建可视化节点，这些节点可以在Action Graph中使用。创建自定义轮式机器人控制器需要以下三个文件：

1. **控制器逻辑类** - 实现具体的控制算法
2. **OGN节点定义文件(.ogn)** - 定义节点的输入、输出和元数据
3. **OGN节点实现文件(.py)** - 将控制器逻辑包装为OGN节点

## 文件结构

创建自定义控制器需要在以下目录中添加文件：

```
source/extensions/isaacsim.robot.wheeled_robots/
├── python/
│   ├── controllers/
│   │   └── your_custom_controller.py          # 控制器逻辑
│   └── nodes/
│       ├── OgnYourCustomController.ogn        # 节点定义
│       └── OgnYourCustomController.py         # 节点实现
```

## 步骤1：创建控制器逻辑

在`python/controllers/`目录下创建控制器逻辑文件。控制器类必须继承`BaseController`并实现`forward`方法。

**文件命名规范：** `your_controller_name.py`

**核心要求：**
- 继承`BaseController`
- 实现`forward()`方法
- 返回`ArticulationAction`对象

## 步骤2：创建OGN节点定义

在`python/nodes/`目录下创建OGN节点定义文件。

**文件命名规范：** `OgnYourControllerName.ogn`

**关键要素：**
- JSON格式配置文件
- 定义输入输出接口
- 设置UI显示名称
- 指定节点分类为`isaacWheeledRobots`

## 步骤3：创建OGN节点实现

在`python/nodes/`目录下创建OGN节点Python实现文件。

**文件命名规范：** `OgnYourControllerName.py`

**核心组件：**
- 内部状态类（继承`BaseResetNode`）
- 主节点类（包含静态方法）
- Database类导入

## 步骤4：构建项目

运行构建脚本生成Database文件：

```bash
# 在Isaac Sim根目录下
./build.sh 
```

构建成功后，重启Isaac Sim即可在Action Graph的Isaac Wheeled Robots分类下找到新的控制器节点。

## 控制器模板

以下是一个完整的控制器模板，可以作为创建新控制器的起点：

### 模板1：控制器逻辑 (`your_controller_name.py`)

```python
# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import numpy as np
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction

class YourControllerName(BaseController):
    """
    自定义控制器描述
    
    Args:
        name (str): 控制器名称
        # 添加你的参数说明
    """
    
    def __init__(
        self,
        name: str,
        # 添加你的参数
        **kwargs
    ) -> None:
        super().__init__(name)
        
        # 初始化你的参数
        # self.parameter1 = parameter1
        # self.parameter2 = parameter2

    def forward(self, command: np.ndarray) -> ArticulationAction:
        """
        计算控制输出
        
        Args:
            command (np.ndarray): 输入命令数组
        
        Returns:
            ArticulationAction: 控制动作
        """
        if isinstance(command, list):
            command = np.array(command)
            
        # 参数验证
        if len(command) < 2:  # 根据你的需求调整
            carb.log_warn(f"命令长度错误: {len(command)}")
            return ArticulationAction()
        
        # 提取命令参数
        # linear_vel = command[0]
        # angular_vel = command[1]
        
        # 实现你的控制算法
        wheel_velocities, steering_angles = self._compute_control(command)
        
        return ArticulationAction(
            joint_velocities=wheel_velocities,
            joint_positions=steering_angles
        )
    
    def _compute_control(self, command):
        """实现具体的控制算法"""
        # 在这里实现你的控制逻辑
        
        # 示例：返回零值
        num_wheels = 4  # 根据你的机器人调整
        wheel_velocities = np.zeros(num_wheels)
        steering_angles = np.zeros(num_wheels)
        
        return wheel_velocities, steering_angles
```

### 模板2：OGN节点定义 (`OgnYourControllerName.ogn`)

```json
{
    "YourControllerName": {
        "version": 1,
        "description": "Your Custom Controller Description",
        "language": "Python",
        "categories": {
            "isaacWheeledRobots": "Your controller description for Isaac Sim"
        },
        "metadata": {
            "uiName": "Your Controller Name"
        },
        "inputs": {
            "execIn": {
                "type": "execution",
                "description": "The input execution"
            },
            "chassisLinearVel": {
                "type": "vectord[3]",
                "description": "Chassis linear velocity [x, y, z]",
                "default": [0.0, 0.0, 0.0]
            },
            "chassisAngularVel": {
                "type": "vectord[3]",
                "description": "Chassis angular velocity [x, y, z]",
                "default": [0.0, 0.0, 0.0]
            },
            "dt": {
                "type": "double",
                "description": "Delta time for simulation step"
            }
        },
        "outputs": {
            "execOut": {
                "type": "execution",
                "description": "The output execution"
            },
            "wheelVelocities": {
                "type": "double[]",
                "description": "Angular velocities for wheels in rad/s"
            },
            "steeringAngles": {
                "type": "double[]",
                "description": "Steering angles for wheels in radians"
            }
        }
    }
}
```

### 模板3：OGN节点实现 (`OgnYourControllerName.py`)

```python
# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.your_controller_name import YourControllerName
from isaacsim.robot.wheeled_robots.ogn.OgnYourControllerNameDatabase import OgnYourControllerNameDatabase


class OgnYourControllerNameInternalState(BaseResetNode):
    """内部状态类"""
    
    def __init__(self):
        # 初始化参数
        # self.parameter1 = 0.0
        # self.parameter2 = 0.0
        
        # 控制器实例
        self.controller_handle = None
        self.node = None
        self.graph_id = None
        self.initialized = False
        
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        """初始化控制器实例"""
        self.controller_handle = YourControllerName(
            name="your_controller_instance",
            # 传递你的参数
            # parameter1=self.parameter1,
            # parameter2=self.parameter2,
        )
        
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        """转发控制命令"""
        return self.controller_handle.forward(command)

    def custom_reset(self):
        """重置输出"""
        # 根据你的输出调整
        self.node.get_attribute("outputs:wheelVelocities").set([0.0, 0.0, 0.0, 0.0])
        self.node.get_attribute("outputs:steeringAngles").set([0.0, 0.0, 0.0, 0.0])


class OgnYourControllerName:
    """OGN节点主类"""
    
    @staticmethod
    def init_instance(node, graph_instance_id):
        """初始化节点实例"""
        state = OgnYourControllerNameDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def internal_state():
        """返回内部状态类实例"""
        return OgnYourControllerNameInternalState()

    @staticmethod
    def compute(db) -> bool:
        """计算函数"""
        state = db.per_instance_state

        try:
            if not state.initialized:
                # 从输入初始化参数
                # state.parameter1 = db.inputs.parameter1
                # state.parameter2 = db.inputs.parameter2
                
                state.initialize_controller()

            # 提取输入
            chassis_linear_vel = np.array(db.inputs.chassisLinearVel)
            chassis_angular_vel = np.array(db.inputs.chassisAngularVel)
            
            # 构建命令数组（根据你的控制器需求调整）
            command = np.array([
                chassis_linear_vel[0],  # X线性速度
                chassis_angular_vel[2], # Z角速度（偏航）
                db.inputs.dt
            ])

            # 执行控制器
            actions = state.forward(command)

            # 设置输出
            if actions.joint_velocities is not None:
                db.outputs.wheelVelocities = actions.joint_velocities.tolist()
            
            if actions.joint_positions is not None:
                db.outputs.steeringAngles = actions.joint_positions.tolist()

        except Exception as error:
            carb.log_error(f"控制器计算错误: {error}")
            # 设置安全的默认输出
            db.outputs.wheelVelocities = [0.0, 0.0, 0.0, 0.0]
            db.outputs.steeringAngles = [0.0, 0.0, 0.0, 0.0]

        # 设置执行输出
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        """释放节点实例"""
        try:
            state = OgnYourControllerNameDatabase.get_internal_state(node, graph_instance_id)
            if state is not None:
                state.reset()
                state.initialized = False
        except Exception:
            pass
```

## 使用模板的步骤

1. **复制模板文件**：将上述三个模板复制到对应目录
2. **替换名称**：将所有`YourControllerName`替换为你的控制器名称
3. **修改参数**：根据你的控制器需求添加输入参数
4. **实现算法**：在`_compute_control`方法中实现具体的控制逻辑
5. **调整输入输出**：根据需要修改OGN文件中的输入输出定义
6. **构建测试**：运行构建脚本并在Isaac Sim中测试

---

**注意：** 
- 确保所有文件名和类名保持一致的命名规范
- 构建后必须重启Isaac Sim才能看到新节点
- 节点将出现在Action Graph的"Isaac Wheeled Robots"分类下
