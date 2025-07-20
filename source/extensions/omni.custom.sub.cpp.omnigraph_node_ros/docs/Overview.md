```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`,{ref}`changelog_omni_custom_sub_cpp_omnigraph_node_ros`
```

(ext_omni_custom_sub_cpp_omnigraph_node_ros)=

# Overview

一个自定义的 C++ 扩展，用于在 Isaac Sim 中创建 ROS2 自定义消息订阅节点。

本扩展演示了如何：
- 创建订阅 ROS2 自定义消息的 OmniGraph 节点
- 处理复杂的消息结构（如 mm_msgs/RobotCmd）
- 将 ROS2 数据集成到 Isaac Sim 的计算图系统中

## 主要功能

- **ROS2 自定义消息订阅**：支持订阅 `mm_msgs::msg::RobotCmd` 消息
- **数据解析和转换**：将 ROS2 消息字段映射到 OmniGraph 输出端口
- **实时数据流**：提供机器人指令数据给 Isaac Sim 仿真环境

## 包含的节点

### ROS2SubscribeRobotCmdNode
订阅 `robot_cmd` 话题并输出以下数据：
- **yaw**: 偏航角度
- **chassisLinearVel**: 底盘线速度
- **chassisAngularVel**: 底盘角速度
- **gripperCmd**: 机械臂关节指令
- **timestamp**: 消息时间戳
- **messageReceived**: 消息接收标志

有关详细的实现流程，请参阅 [创建自定义ROS2订阅节点指南](ROS2CustomNodeGuide.md)。
