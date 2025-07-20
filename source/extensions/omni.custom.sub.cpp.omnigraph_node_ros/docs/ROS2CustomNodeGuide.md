# 创建自定义ROS2订阅节点的OmniGraph实现指南

本指南详细描述了如何在 Isaac Sim 中创建一个订阅 ROS2 自定义消息的 OmniGraph 节点。以 `mm_msgs::msg::RobotCmd` 消息为例，展示从零开始构建一个完整的ROS2订阅节点的全过程。

## 概述

通过本指南，你将学会：
- 如何设计和实现ROS2自定义消息的订阅节点
- 如何配置依赖项和构建系统
- 如何在OmniGraph中使用自定义ROS2节点
- 如何调试和故障排除

## 目录

1. [前置条件](#前置条件)
2. [消息结构分析](#消息结构分析)
3. [项目结构准备](#项目结构准备)
4. [依赖配置](#依赖配置)
5. [节点定义（OGN文件）](#节点定义ogn文件)
6. [C++实现](#c实现)
7. [构建配置](#构建配置)
8. [编译和测试](#编译和测试)
9. [在Isaac Sim中使用](#在isaac-sim中使用)
10. [故障排除](#故障排除)

## 前置条件

### 环境要求
- **Isaac Sim**: 2023.1.1 或更高版本
- **ROS2**: Humble 或 Foxy
- **操作系统**: Ubuntu 20.04/22.04
- **编译器**: C++17 支持
- **自定义消息包**: 已编译的ROS2消息包 (如 `mm_msgs`)

### 依赖检查
```bash
# 1. 检查 ROS2 环境
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO

# 2. 检查自定义消息包
ros2 pkg list | grep mm_msgs

# 3. 检查消息定义
ros2 interface show mm_msgs/msg/RobotCmd

# 4. 验证消息依赖
ros2 interface show std_msgs/msg/Header
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show trajectory_msgs/msg/JointTrajectory
```

### ROS2 工作空间设置
确保你的自定义消息包已正确编译：
```bash
# 假设你的工作空间在 ~/isaac_ws
cd ~/isaac_ws
colcon build --packages-select mm_msgs
source install/setup.bash
```

## 消息结构分析

在开始实现之前，需要分析目标ROS2消息的结构。以 `mm_msgs::msg::RobotCmd` 为例：

### RobotCmd 消息结构
```bash
# 查看消息定义
ros2 interface show mm_msgs/msg/RobotCmd
```

假设消息结构如下：
```yaml
std_msgs/Header header
float32 yaw
geometry_msgs/Twist chassis_cmd
float32 gripper_cmd
```

### 依赖消息分析
- `std_msgs/Header`: 包含时间戳和frame_id
- `geometry_msgs/Twist`: 包含线性和角速度
- `float32`: 基本数据类型

这种分析帮助我们：
1. 确定需要的依赖包
2. 设计OmniGraph节点的输出端口
3. 实现正确的数据转换

## 项目结构准备

### 1. 创建扩展目录结构
```
omni.custom.sub.cpp.omnigraph_node_ros/
├── config/
│   └── extension.toml          # 扩展配置文件
├── data/
│   ├── icon.png               # 扩展图标
│   └── preview.png            # 预览图片
├── docs/
│   ├── Overview.md            # 扩展概述
│   ├── ROS2CustomNodeGuide.md # 详细实现指南
│      
├── omni/                      # Python模块目录
│   └── custom/
│       └── sub/
│           └── cpp/
│               └── omnigraph_node_ros/
│                   └── __init__.py
├── plugins/                   # C++插件目录
│   ├── nodes/                 # 节点实现
│   │   ├── ROS2SubscribeRobotCmdNode.ogn  # 节点定义
│   │   ├── ROS2SubscribeRobotCmdNode.cpp  # 节点实现
│   │   └── icons/
│   │       └── isaac-sim.svg
│   └── omni.custom.sub.cpp.omnigraph_node_ros/
│       └── CustomSubOmniGraphNodeROSExtension.cpp  # 扩展主入口
└── premake5.lua               # 构建配置
```

### 2. 创建基础目录
```bash
# 在 Isaac Sim kit-extension-template-cpp 目录下
mkdir -p source/extensions/omni.custom.sub.cpp.omnigraph_node_ros/{config,data,docs,omni/custom/sub/cpp/omnigraph_node_ros,plugins/nodes/icons,plugins/omni.custom.sub.cpp.omnigraph_node_ros}
```

### 3. 扩展配置文件
创建 `config/extension.toml`：
```toml
[package]
version = "1.0.3"
title = "Custom ROS2 OGN Example Extension"
description = "Demonstrates how to create a ROS2 C++ node for OmniGraph"
category = "Custom"
keywords = ["custom", "C++", "cpp", "Graph", "Node", "OmniGraph", "ROS2"]
icon = "data/icon.png"
preview_image = "data/preview.png"
changelog = "docs/CHANGELOG.md"
readme = "docs/README.md"

[dependencies]
"omni.graph.core" = {}
"omni.graph.tools" = {}

[[python.module]]
name = "omni.custom.sub.cpp.omnigraph_node_ros"

[[native.plugin]]
path = "bin/*.plugin"

[documentation]
pages = [
    "docs/Overview.md",
    "docs/ROS2CustomNodeGuide.md",
    "docs/QuickStart.md",
    "docs/Troubleshooting.md",
]
```
│                   └── __init__.py
├── plugins/
│   ├── nodes/
│   │   ├── ROS2CustomMessageNode.ogn
│   │   ├── ROS2CustomMessageNode.cpp
│   │   └── icons/
│   └── omni.custom.sub.cpp.omnigraph_node_ros/
│       └── CustomSubOmniGraphNodeROSExtension.cpp
└── premake5.lua
```

### 2. 配置扩展信息
编辑 `config/extension.toml`：
```toml
[package]
version = "1.0.3"
category = "simulation"
title = "Custom ROS2 Subscriber OmniGraph Node"
description = "Custom C++ extension for subscribing to ROS2 custom messages in OmniGraph"
authors = ["NVIDIA"]
repository = ""
keywords = ["ros2", "omnigraph", "subscriber", "custom", "messages"]

[dependencies]
"omni.graph.core" = {}
"omni.graph.tools" = {}

[[python.module]]
name = "omni.custom.sub.cpp.omnigraph_node_ros"
```

## 依赖配置

### 1. 系统级ROS2依赖配置
在项目根目录的 `deps/kit-sdk-deps.packman.xml` 中添加必要的ROS2依赖：

```xml
<project toolsVersion="5.0">
  <!-- ...existing dependencies... -->

  <!-- System ROS2 installation -->
  <dependency name="system_ros" linkPath="../_build/target-deps/system_ros" tags="${config}">
      <source path="/opt/ros/humble"/>
  </dependency>

  <!-- Additional ROS workspace (for tutorial_interfaces) -->
  <dependency name="additional_ros_workspace" linkPath="../_build/target-deps/additional_ros" tags="${config}">
      <source path="/home/user/isaac_ws/install/tutorial_interfaces" />
  </dependency>

  <!-- Custom mm_msgs package -->
  <dependency name="mm_msgs" linkPath="../_build/target-deps/mm_msgs" tags="${config}">
      <source path="/home/user/isaac_ws/install/mm_msgs" />
  </dependency>
</project>
```

**重要提示**:
- 将路径修改为你的实际ROS2工作空间路径
- 确保所有依赖包都已正确编译

### 2. Python模块初始化
创建必要的 `__init__.py` 文件：

```python
# omni/__init__.py
# omni package

# omni/custom/__init__.py
# omni.custom package

# omni/custom/sub/__init__.py
# omni.custom.sub package

# omni/custom/sub/cpp/__init__.py
# omni.custom.sub.cpp package

# omni/custom/sub/cpp/omnigraph_node_ros/__init__.py
## Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
##
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.
##

# This file is needed so tests don't fail.
```

## 节点定义（OGN文件）

OGN (OmniGraph Node) 文件定义了节点的接口，包括输入端口、输出端口和元数据。

### 1. 创建 ROS2SubscribeRobotCmdNode.ogn

在 `plugins/nodes/ROS2SubscribeRobotCmdNode.ogn` 中定义节点：

```json
{
    "ROS2SubscribeRobotCmdNode": {
        "version": 1,
        "icon": "icons/isaac-sim.svg",
        "description": [
            "This node subscribes to robot_cmd topic and outputs robot command data"
        ],
        "metadata": {
            "uiName": "ROS2 Subscribe Robot Command"
        },
        "categories": ["tutorials"],
        "inputs": {
            "execIn": {
                "type": "execution",
                "description": "The input execution port."
            }
        },
        "outputs": {
            "execOut": {
                "type": "execution",
                "description": "The output execution port."
            },
            "yaw": {
                "type": "float",
                "description": "Yaw angle from robot command"
            },
            "chassisLinearVel": {
                "type": "float[3]",
                "description": "Chassis linear velocity [x, y, z]"
            },
            "chassisAngularVel": {
                "type": "float[3]",
                "description": "Chassis angular velocity [x, y, z]"
            },
            "gripperCmd": {
                "type": "float",
                "description": "Gripper command value"
            },
            "timestamp": {
                "type": "double",
                "description": "Message timestamp in seconds"
            },
            "messageReceived": {
                "type": "bool",
                "description": "Whether a message was received this frame"
            }
        }
    }
}
```

### 2. OGN文件设计要点

#### 输入端口设计
- `execIn`: 执行端口，触发节点计算
- 可以添加配置参数，如话题名称、QoS设置等

#### 输出端口设计
根据ROS2消息结构设计输出：
- **基本类型**: `float`, `double`, `int`, `bool`
- **数组类型**: `float[3]` 用于3D向量
- **动态数组**: `float[]` 用于可变长度数据
- **执行端口**: `execution` 用于控制流

#### 数据类型映射
| ROS2 类型 | OmniGraph 类型 | 说明 |
|-----------|----------------|------|
| `float32` | `float` | 32位浮点数 |
| `float64` | `double` | 64位浮点数 |
| `int32` | `int` | 32位整数 |
| `bool` | `bool` | 布尔值 |
| `geometry_msgs/Vector3` | `float[3]` | 3D向量 |
| `std_msgs/Header.stamp` | `double` | 时间戳转换为秒 |

### 3. 节点图标
在 `plugins/nodes/icons/` 目录下放置节点图标文件 `isaac-sim.svg`。这将在OmniGraph编辑器中显示。

## C++实现

C++实现是节点的核心逻辑，负责ROS2消息的订阅和数据处理。

### 1. 头文件包含

在 `plugins/nodes/ROS2SubscribeRobotCmdNode.cpp` 的开头包含必要的头文件：

```cpp
#include <ROS2SubscribeRobotCmdNodeDatabase.h>
#include <string>
#include <memory>

// Include the mm_msgs RobotCmd message
#include "mm_msgs/msg/robot_cmd.h"

// ROS includes for creating nodes, subscribers etc.
#include "rcl/rcl.h"
#include "rcl/subscription.h"

// Helpers to explicit shorten names you know you will use
using omni::graph::core::Type;
using omni::graph::core::BaseDataType;
```

### 2. 命名空间和类定义

```cpp
namespace omni {
namespace custom {  // 注意：使用自定义命名空间
namespace sub {
namespace cpp {
namespace omnigraph_node_ros {

class ROS2SubscribeRobotCmdNode
{
public:
    static bool compute(ROS2SubscribeRobotCmdNodeDatabase& db);
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId);

private:
    // ROS2 相关状态变量
    rcl_subscription_t my_sub;
    rcl_node_t my_node;
    rcl_context_t context;
    rcl_node_options_t node_ops;
    rcl_init_options_t init_options;
    rcl_allocator_t allocator;
    rcl_subscription_options_t sub_options;
    bool sub_created {false};
    bool message_received {false};
};
```

### 3. compute() 方法实现

这是节点的主要计算方法，在每个OmniGraph执行周期都会被调用：

```cpp
static bool compute(ROS2SubscribeRobotCmdNodeDatabase& db)
{
    auto& state = db.internalState<ROS2SubscribeRobotCmdNode>();

    // 1. 初始化ROS2订阅者（仅执行一次）
    if(!state.sub_created)
    {
        // 初始化ROS2上下文
        state.context = rcl_get_zero_initialized_context();
        state.init_options = rcl_get_zero_initialized_init_options();
        state.allocator = rcl_get_default_allocator();

        // 创建初始化选项
        rcl_ret_t rc = rcl_init_options_init(&state.init_options, state.allocator);
        if (rc != RCL_RET_OK) {
            printf("Error rcl_init_options_init.\n");
            return false;
        }

        // 创建上下文
        rc = rcl_init(0, nullptr, &state.init_options, &state.context);
        if (rc != RCL_RET_OK) {
            printf("Error in rcl_init.\n");
            return false;
        }

        // 创建ROS2节点
        state.my_node = rcl_get_zero_initialized_node();
        state.node_ops = rcl_node_get_default_options();
        rc = rcl_node_init(&state.my_node, "robot_cmd_subscriber",
                          "mm_robot_cmd_sub", &state.context, &state.node_ops);
        if (rc != RCL_RET_OK) {
            printf("Error in rcl_node_init\n");
            return false;
        }

        // 创建订阅者
        const char * topic_name = "robot_cmd";
        const rosidl_message_type_support_t * my_type_support =
            ROSIDL_GET_MSG_TYPE_SUPPORT(mm_msgs, msg, RobotCmd);

        state.sub_options = rcl_subscription_get_default_options();
        rc = rcl_subscription_init(&state.my_sub, &state.my_node,
                                  my_type_support, topic_name, &state.sub_options);
        if (RCL_RET_OK != rc) {
            printf("Error in rcl_subscription_init %s.\n", topic_name);
            return false;
        }

        state.sub_created = true;
        printf("ROS2 Robot Command subscriber initialized on topic: %s\n", topic_name);
    }

    // 2. 尝试接收消息
    mm_msgs__msg__RobotCmd * ros_msg = mm_msgs__msg__RobotCmd__create();
    rmw_message_info_t message_info;

    rcl_ret_t rc = rcl_take(&state.my_sub, ros_msg, &message_info, nullptr);

    if (rc == RCL_RET_OK) {
        // 3. 消息接收成功，设置输出
        db.outputs.yaw() = ros_msg->yaw;

        // 设置底盘线速度
        auto chassisLinearVel = db.outputs.chassisLinearVel();
        chassisLinearVel[0] = static_cast<float>(ros_msg->chassis_cmd.linear.x);
        chassisLinearVel[1] = static_cast<float>(ros_msg->chassis_cmd.linear.y);
        chassisLinearVel[2] = static_cast<float>(ros_msg->chassis_cmd.linear.z);

        // 设置底盘角速度
        auto chassisAngularVel = db.outputs.chassisAngularVel();
        chassisAngularVel[0] = static_cast<float>(ros_msg->chassis_cmd.angular.x);
        chassisAngularVel[1] = static_cast<float>(ros_msg->chassis_cmd.angular.y);
        chassisAngularVel[2] = static_cast<float>(ros_msg->chassis_cmd.angular.z);

        // 设置夹爪指令
        db.outputs.gripperCmd() = ros_msg->gripper_cmd;

        // 设置时间戳
        db.outputs.timestamp() = static_cast<double>(ros_msg->header.stamp.sec) +
                                 static_cast<double>(ros_msg->header.stamp.nanosec) * 1e-9;

        // 设置消息接收标志
        db.outputs.messageReceived() = true;

        // 可选：打印调试信息
        printf("Received RobotCmd: yaw=%.2f, linear_vel=[%.2f,%.2f,%.2f]\n",
               ros_msg->yaw, ros_msg->chassis_cmd.linear.x,
               ros_msg->chassis_cmd.linear.y, ros_msg->chassis_cmd.linear.z);
    }
    else if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        // 无消息可用
        db.outputs.messageReceived() = false;
    }
    else {
        // 错误发生
        printf("Error taking RobotCmd message: %d\n", rc);
        db.outputs.messageReceived() = false;
    }

    // 清理消息
    mm_msgs__msg__RobotCmd__destroy(ros_msg);

    return true;  // 始终返回true以继续执行
}
```

### 4. releaseInstance() 方法实现

清理资源的方法，在节点被销毁时调用：

```cpp
static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
{
    auto& state = ROS2SubscribeRobotCmdNodeDatabase::sPerInstanceState<ROS2SubscribeRobotCmdNode>(nodeObj, instanceId);

    if (state.sub_created) {
        // 清理订阅者
        rcl_ret_t rc = rcl_subscription_fini(&state.my_sub, &state.my_node);
        if (rc != RCL_RET_OK) {
            printf("Failed to finalize subscriber: %d\n", rc);
        }

        // 清理节点
        rc = rcl_node_fini(&state.my_node);
        if (rc != RCL_RET_OK) {
            printf("Failed to finalize node: %d\n", rc);
        }

        // 清理初始化选项
        rc = rcl_init_options_fini(&state.init_options);
        if (rc != RCL_RET_OK) {
            printf("Failed to finalize init options: %d\n", rc);
        }

        // 关闭上下文
        rc = rcl_shutdown(&state.context);
        if (rc != RCL_RET_OK) {
            printf("Failed to shutdown context: %d\n", rc);
        }

        state.sub_created = false;
        printf("ROS2 Robot Command subscriber cleaned up\n");
    }
}
```

### 5. 节点注册宏

在文件末尾添加节点注册宏：

```cpp
// 这个宏提供OmniGraph自动注册和注销节点类型定义所需的信息
REGISTER_OGN_NODE()

} // omnigraph_node_ros
} // cpp
} // sub
} // custom
} // omni
```

### 6. 实现要点

#### 状态管理
- 使用 `internalState<>()` 管理节点的持久状态
- `sub_created` 标志确保初始化只执行一次
- 正确的资源清理防止内存泄漏

#### 数据类型转换
- ROS2的 `float64` 转换为OmniGraph的 `double`
- ROS2的 `float32` 转换为OmniGraph的 `float`
- 向量数据使用数组索引访问

#### 错误处理
- 检查所有ROS2 API的返回值
- 提供有意义的错误信息
- 优雅地处理初始化失败

#### 性能考虑
- 避免在每帧都进行重复的初始化
- 使用 `rcl_take()` 的非阻塞方式
- 适当的调试输出（可在发布版本中禁用）

## 构建配置

### 配置 premake5.lua
```lua
-- 基本扩展信息
local ext = get_current_extension_info()
project_ext(ext)

-- OGN 项目配置
local ogn = get_ogn_project_information(ext, "omni/custom/sub/cpp/omnigraph_node_ros")

-- 构建 OGN 文件
project_ext_ogn(ext, ogn)

-- 构建 C++ 插件
project_ext_plugin(ext, "omni.custom.sub.cpp.omnigraph_node_ros.plugin")
    add_files("source", "plugins/omni.custom.sub.cpp.omnigraph_node_ros")
    add_files("nodes", "plugins/nodes")

    -- 添加 OGN 依赖
    add_ogn_dependencies(ogn)

    -- ROS2 头文件路径
    includedirs {
        "%{target_deps}/system_ros/include/std_msgs",
        "%{target_deps}/system_ros/include/geometry_msgs",
        "%{target_deps}/system_ros/include/trajectory_msgs",
        "%{target_deps}/system_ros/include/builtin_interfaces",
        "%{target_deps}/system_ros/include/rosidl_runtime_c",
        "%{target_deps}/system_ros/include/rcl",
        "%{target_deps}/system_ros/include/rcutils",
        "%{target_deps}/system_ros/include/rmw",
        "%{target_deps}/mm_msgs/include/mm_msgs",
    }

    -- 库文件路径
    libdirs {
        "%{target_deps}/system_ros/lib",
        "%{target_deps}/mm_msgs/lib",
    }

    -- 链接库
    links {
        "rosidl_runtime_c", "rcutils", "rcl", "rmw",
        "std_msgs__rosidl_typesupport_c", "std_msgs__rosidl_generator_c",
        "geometry_msgs__rosidl_typesupport_c", "geometry_msgs__rosidl_generator_c",
        "trajectory_msgs__rosidl_typesupport_c", "trajectory_msgs__rosidl_generator_c",
        "builtin_interfaces__rosidl_typesupport_c", "builtin_interfaces__rosidl_generator_c",
        "mm_msgs__rosidl_typesupport_c", "mm_msgs__rosidl_generator_c",
    }

    filter { "system:linux" }
        linkoptions { "-Wl,--export-dynamic" }

    cppdialect "C++17"
```

## 编译和测试

### 1. 编译扩展
```bash
# 在项目根目录执行
cd /path/to/kit-extension-template-cpp
./build.sh
```

### 2. 验证构建输出
检查生成的文件：
```bash
ls _build/linux-x86_64/release/exts/omni.custom.sub.cpp.omnigraph_node_ros/bin/
# 应该看到: libomni.custom.sub.cpp.omnigraph_node_ros.plugin.so
```

### 3. 测试 ROS2 连接
在终端中启动测试发布者：
```bash
# 终端1: 启动 Isaac Sim
./isaac-sim.sh

# 终端2: 发布测试消息
source /opt/ros/humble/setup.bash
ros2 topic pub /robot_cmd mm_msgs/msg/RobotCmd '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"},
  yaw: 1.57,
  chassis_cmd: {
    linear: {x: 0.5, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.2}
  },
  gripper_cmd: {
    points: [{positions: [0.1, 0.2, 0.3]}]
  }
}'
```

## 使用方法

### 1. 在 Isaac Sim 中使用节点

1. 打开 Isaac Sim
2. 启用扩展：
   - Window → Extensions
   - 搜索 "omni.custom.sub.cpp.omnigraph_node_ros"
   - 启用扩展

3. 创建 OmniGraph：
   - Window → Visual Scripting → Action Graph
   - 添加 "ROS2 Subscribe Robot Command" 节点

### 2. 连接输出端口

将节点的输出连接到其他节点：
- `yaw` → 机器人旋转控制
- `chassisLinearVel/chassisAngularVel` → 移动控制
- `gripperCmd` → 机械臂控制
- `messageReceived` → 状态指示

### 3. 监控数据流

使用 Isaac Sim 的图形界面监控：
- 节点属性面板显示实时数值
- 可以添加调试输出节点查看数据

## 故障排除

### 常见问题

1. **编译错误 - 缺少头文件**
   ```
   解决方案: 检查 premake5.lua 中的 includedirs 配置
   ```

2. **链接错误 - 找不到库文件**
   ```
   解决方案: 验证 ROS2 环境和 mm_msgs 包是否正确安装
   ```

3. **运行时错误 - ROS2 节点初始化失败**
   ```
   解决方案: 确保 ROS2 环境变量已设置，使用 source /opt/ros/humble/setup.bash
   ```

4. **没有接收到消息**
   ```
   解决方案: 检查话题名称、消息类型和网络连接
   ```

### 调试技巧

1. **启用详细日志**：
   ```cpp
   RCLCPP_INFO(s_node->get_logger(), "Message received: yaw=%f", msg->yaw);
   ```

2. **使用 ROS2 工具检查**：
   ```bash
   ros2 topic list
   ros2 topic echo /robot_cmd
   ros2 node list
   ```

3. **检查扩展状态**：
   - Isaac Sim 扩展管理器中查看扩展状态
   - 控制台查看错误信息

## 扩展和定制

### 与4WD4WS控制器集成

基于之前创建的ROS2订阅节点，您可以进一步集成自定义的4WD4WS控制器：

#### 1. 4WD4WS控制器框架
我们创建了一个完整的四轮驱动四轮转向控制器扩展，位于：
```
source/extensions/omni.custom.4wd4ws.controller/
```

#### 2. 数据流连接
在OmniGraph中建立以下连接：

```
[ROS2 Subscribe Robot Cmd] → [4WD4WS Controller] → [Articulation Controller]
     ↓ chassisLinearVel           ↓ wheelVelocities
     ↓ chassisAngularVel          ↓ steeringAngles
```

#### 3. 连接步骤
1. 添加ROS2订阅节点（之前创建的）
2. 添加4WD4WS控制器节点
3. 连接数据端口：
   - `chassisLinearVel` → `linearVelocity`
   - `chassisAngularVel` → `angularVelocity`
4. 配置车辆参数（轴距、轮距等）
5. 将输出连接到Isaac Sim的关节控制器

#### 4. 完整的ROS2到Isaac Sim控制链
```mermaid
graph LR
    A[ROS2 Publisher] --> B[ROS2 Topic: /robot_cmd]
    B --> C[ROS2 Subscribe Node]
    C --> D[4WD4WS Controller]
    D --> E[Isaac Sim Vehicle]
```

### 添加新的消息类型

1. 更新 OGN 文件添加新的输出端口
2. 修改 C++ 实现处理新字段
3. 更新 premake5.lua 添加新的依赖库
4. 重新编译扩展

### 性能优化

1. **消息缓存**：避免频繁的内存分配
2. **异步处理**：使用独立线程处理 ROS2 回调
3. **数据过滤**：根据需要过滤不必要的数据

### 多节点支持

可以扩展此模式创建多个不同类型的 ROS2 订阅节点，每个节点处理不同的消息类型。

---

*此指南基于 Isaac Sim 4.5 和 ROS2 Humble。对于其他版本，某些步骤可能需要相应调整。*
