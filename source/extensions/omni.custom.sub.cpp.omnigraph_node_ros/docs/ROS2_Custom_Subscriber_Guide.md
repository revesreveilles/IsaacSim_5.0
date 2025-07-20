# 创建自定义ROS2消息订阅OmniGraph节点指南

## 概述

本指南详细介绍了如何在 Isaac Sim 扩展中，从零开始创建一个订阅**自定义ROS2消息**的C++ OmniGraph节点。

## 前提条件

1.  **Isaac Sim 2023.1.1+**
2.  **ROS2 Humble** 已安装并配置好工作空间
3.  **自定义ROS2消息包** (例如 `mm_msgs`) 已在您的ROS2工作空间中成功编译

## 核心流程

创建自定义节点的流程分为五步：
1.  **项目结构** - 搭建扩展目录和文件
2.  **节点定义(.ogn)** - 设计节点的输入输出端口
3.  **C++实现(.cpp)** - 编写ROS2订阅和数据处理逻辑
4.  **构建配置(premake5.lua)** - 配置编译依赖和链接
5.  **编译与测试** - 构建扩展并在Isaac Sim中验证

---

## 快速开始模板

使用此模板快速创建您自己的节点。假设您的自定义消息包名为 `my_msgs`，消息名为 `MyMessage`。

### 1. 目录结构

在您的扩展中创建以下文件：
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
│   ├── QuickStart.md          # 快速入门
│   └── Troubleshooting.md     # 故障排除
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

### 2. 节点定义模板 (`MyNode.ogn`)

```json
{
    "MyNode": {
        "version": 1,
        "description": "订阅 MyMessage 消息",
        "language": "C++",
        "categories": { "My ROS2 Nodes": "自定义ROS2节点" },
        "metadata": { "uiName": "ROS2 Subscribe MyMessage" },
        "inputs": {
            "execIn": { "type": "execution" },
            "topicName": { "type": "string", "default": "my_topic" }
        },
        "outputs": {
            "execOut": { "type": "execution" },
            "messageReceived": { "type": "bool", "default": false },
            "myData": { "type": "double", "default": 0.0 }
        }
    }
}
```

### 3. C++实现模板 (`MyNode.cpp`)

```cpp
#include <MyNodeDatabase.h>
#include <rclcpp/rclcpp.hpp>
#include <my_msgs/msg/my_message.hpp> // 替换为您的消息头文件

class MyNode
{
public:
    // OmniGraph 节点计算函数
    static bool compute(MyNodeDatabase& db)
    {
        // ... ROS2初始化和订阅逻辑 ...

        // 示例：当收到新消息时更新输出端口
        if (/* new message received */) {
            // db.outputs.myData() = received_message.data;
            db.outputs.messageReceived() = true;
        } else {
            db.outputs.messageReceived() = false;
        }
        return true;
    }
};

// 注册节点
REGISTER_OGN_NODE()
```

### 4. 构建配置模板 (`premake5.lua`)

```lua
-- 添加头文件包含路径
includedirs {
    "%{target_deps}/my_msgs/include/my_msgs",
    -- 其他必要的ROS2头文件
}

-- 添加链接库
links {
    "my_msgs__rosidl_typesupport_c",
    "my_msgs__rosidl_generator_c",
    -- 其他必要的ROS2库
}
```

---

## 详细步骤与参数说明

### 步骤1: 项目结构

标准的节点文件位于扩展的 `plugins/nodes/` 目录下。

-   `ROS2SubscribeRobotCmdNode.ogn`: **节点接口定义**。定义节点的输入/输出端口、名称、分类等，是OmniGraph识别节点的依据。
-   `ROS2SubscribeRobotCmdNode.cpp`: **节点核心逻辑**。实现ROS2初始化、消息订阅、数据处理和到输出端口的映射。

### 步骤2: 节点定义 (`.ogn` 文件)

这是节点的“蓝图”，定义了它在OmniGraph中的外观和接口。

**关键参数说明:**

-   `"language": "C++"`: 指明这是一个C++节点。
-   `"categories"`: **节点分类**。用于在OmniGraph节点浏览器中组织节点。
    -   **简单方式**: `"categories": { "My Category": "Description" }`
    -   **分层方式**: 需要配合 `"categoryDefinitions"` 文件，详见 `OmniGraph_Categories_Guide.md`。
-   `"metadata": { "uiName": "My Node Name" }`: **显示名称**。在OmniGraph UI中显示的节点名称。
-   `"inputs"`: **输入端口**。
    -   `"execIn"`: **执行输入**，是标准入口。
    -   可以添加配置端口，如 `"topicName"`，让用户在UI上配置。
-   `"outputs"`: **输出端口**。
    -   `"execOut"`: **执行输出**，用于连接下一个节点。
    -   `"messageReceived"`: **状态标志**，判断是否收到新消息，常用于控制流程。
    -   其他数据端口应与您的ROS2消息字段对应。

### 步骤3: C++ 实现 (`.cpp` 文件)

这是节点功能的核心。

**关键逻辑:**

1.  **ROS2初始化**: `rclcpp::init()`，通常在静态变量中管理，确保只执行一次。
2.  **创建ROS2节点**: `rclcpp::Node::make_shared()`。
3.  **创建订阅者**: `create_subscription()`，指定话题名称、QoS和回调函数。
4.  **消息回调函数**: 当收到消息时，将消息数据存储在静态变量中。
5.  **`compute()` 函数**: OmniGraph每次执行时调用此函数。
    -   检查是否有新消息。
    -   如果有，将存储的消息数据赋值给对应的输出端口 (`db.outputs.myPort() = ...`)。
    -   更新 `messageReceived` 状态。

### 步骤4: 构建配置 (`premake5.lua`)

此文件告诉编译器如何找到您的自定义消息的头文件和库文件。

**关键参数说明:**

-   `includedirs`: **头文件包含路径**。必须指向您自定义消息包编译后生成的 `include` 目录。
    -   `"%{target_deps}/<my_pkg_name>/include/<my_pkg_name>"`
-   `links`: **链接库**。链接到您的自定义消息包的库文件。
    -   通常需要 `"<my_pkg_name>__rosidl_typesupport_c"` 和 `"<my_pkg_name>__rosidl_generator_c"`。
-   `libdirs`: **库文件路径**。指向库文件所在的 `lib` 目录。
    -   `"%{target_deps}/<my_pkg_name>/lib"`

**重要**: 您还需要在 `deps/kit-sdk-deps.packman.xml` 中添加对您自定义消息包的依赖，以便构建系统能找到它。

```xml
<!-- deps/kit-sdk-deps.packman.xml -->
<dependency name="my_msgs" linkPath="../_build/target-deps/my_msgs">
    <source path="/path/to/your/ros_ws/install/my_msgs" />
</dependency>
```

### 步骤5: 编译与测试

1.  **编译**: 在项目根目录运行 `./build.sh`。
2.  **启动Isaac Sim**: `./isaac-sim.sh`。
3.  **启用扩展**: 在 **Window > Extensions** 中找到并启用您的扩展。
4.  **测试**:
    -   在 **Action Graph** 中添加您的节点。
    -   使用 `ros2 topic pub` 命令发布测试消息。
    -   在OmniGraph中连接一个 `Print` 节点到您的数据输出端口，观察控制台输出。

---

## 核心故障排除

-   **编译错误: `fatal error: my_msgs/msg/my_message.h: No such file or directory`**
    -   **原因**: `premake5.lua` 中的 `includedirs` 路径错误，或 `deps.packman.xml` 依赖未配置。
    -   **解决**: 仔细检查路径是否指向您ROS2工作空间中 `install` 目录下的对应包。

-   **链接错误: `undefined reference to 'my_msgs__msg__MyMessage__create'`**
    -   **原因**: `premake5.lua` 中的 `links` 或 `libdirs` 配置错误。
    -   **解决**: 确保链接了正确的库，并且库目录路径正确。

-   **运行时错误: 扩展加载失败或节点在列表中找不到**
    -   **原因**: `extension.toml` 配置错误，或C++插件编译失败。
    -   **解决**: 检查构建日志，并验证 `_build` 目录下是否生成了对应的 `.plugin` 文件。

-   **运行时错误: 无法接收ROS2消息**
    -   **原因**: 话题名称不匹配、QoS不兼容或ROS2网络问题。
    -   **解决**: 使用 `ros2 topic echo <topic_name>` 确认消息正在发布，并检查节点中的话题名称和QoS设置。
