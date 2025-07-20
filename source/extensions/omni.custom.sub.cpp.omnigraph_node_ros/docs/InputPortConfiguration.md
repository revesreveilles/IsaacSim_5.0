# 使用输入端口配置 ROS2 订阅节点

## 概述

`ROS2SubscribeRobotCmdNode` 节点支持通过输入端口动态配置ROS2订阅参数，无需重新编译代码即可修改节点行为。

## 可配置的输入端口

| 端口名称 | 类型 | 默认值 | 描述 |
|----------|------|--------|------|
| `context` | `uint64` | `0` | ROS2上下文指针（通常保持默认值） |
| `nodeNamespace` | `string` | `""` | ROS2节点命名空间（可选） |
| `qosProfile` | `string` | `"default"` | QoS配置文件名称 |
| `queueSize` | `int` | `10` | 订阅队列大小 |
| `topicName` | `string` | `"robot_cmd"` | 要订阅的ROS2话题名称 |

## QoS配置文件

### 预定义配置文件

| 配置文件 | 可靠性 | 持久性 | 适用场景 |
|----------|--------|--------|----------|
| `"default"` | Reliable | Volatile | 通用配置 |
| `"sensor_data"` | Best Effort | Volatile | 传感器数据 |
| `"parameter_events"` | Reliable | Volatile | 参数事件 |
| `"system_default"` | Best Effort | Volatile | 系统默认 |

## 使用示例

### 示例1: 基本话题订阅

```json
{
  "topicName": "my_robot_commands",
  "queueSize": 20
}
```

### 示例2: 带命名空间的配置

```json
{
  "topicName": "robot_cmd",
  "nodeNamespace": "robot1",
  "queueSize": 5,
  "qosProfile": "default"
}
```

### 示例3: 传感器数据配置

```json
{
  "topicName": "sensor_readings",
  "qosProfile": "sensor_data",
  "queueSize": 50
}
```

### 示例4: 系统事件配置

```json
{
  "topicName": "system_events",
  "qosProfile": "parameter_events",
  "queueSize": 100
}
```

## 动态重配置

节点支持运行时重配置。当任何输入参数发生变化时，节点会：

1. **自动检测参数变化**
2. **清理现有订阅者**
3. **使用新参数重新创建订阅者**
4. **继续正常运行**

### 参数变化检测

以下参数的变化会触发订阅者重新创建：
- `topicName`
- `nodeNamespace`
- `queueSize`
- `qosProfile`

注意：`context` 参数通常不需要修改，保持默认值即可。

## 在 OmniGraph 中使用

### 1. 连接常量节点

```
[Constant String] → [topicName] → [ROS2 Subscribe Robot Command]
    "my_topic"
```

### 2. 连接变量节点

```
[Variable] → [topicName] → [ROS2 Subscribe Robot Command]
```

### 3. 条件配置

```
[Branch] → [topicName] → [ROS2 Subscribe Robot Command]
   ↓
[condition] ? "topic_a" : "topic_b"
```

## 实际应用场景

### 1. 多机器人系统

为不同机器人配置不同的命名空间：

```python
# 机器人1
robot1_config = {
    "nodeNamespace": "robot1",
    "topicName": "cmd",
    "queueSize": 10
}

# 机器人2
robot2_config = {
    "nodeNamespace": "robot2",
    "topicName": "cmd",
    "queueSize": 10
}

# 结果话题: /robot1/cmd, /robot2/cmd
```

### 2. 开发vs生产环境

```python
# 开发环境 - 使用调试话题
dev_config = {
    "topicName": "debug_robot_cmd",
    "qosProfile": "sensor_data",  # 最佳努力
    "queueSize": 5
}

# 生产环境 - 使用标准话题
prod_config = {
    "topicName": "robot_cmd",
    "qosProfile": "default",  # 可靠传输
    "queueSize": 20
}
```

### 3. 性能调优

```python
# 高频传感器数据
sensor_config = {
    "topicName": "lidar_data",
    "qosProfile": "sensor_data",  # 最佳努力，低延迟
    "queueSize": 1  # 只保留最新数据
}

# 关键控制指令
control_config = {
    "topicName": "safety_stop",
    "qosProfile": "default",  # 可靠传输
    "queueSize": 100  # 确保不丢失
}
```

## 调试和监控

### 1. 控制台输出

节点在重新配置时会输出详细信息：

```
ROS2 Robot Command subscriber initialized:
  Topic: my_robot_commands
  Node: robot_cmd_subscriber_node
  Namespace: /robot1
  Queue Size: 20
  QoS Profile: default
```

### 2. ROS2 工具验证

```bash
# 检查节点
ros2 node list | grep robot_cmd_subscriber

# 检查话题
ros2 topic list | grep my_robot_commands

# 检查订阅关系
ros2 node info /robot1/robot_cmd_subscriber_node
```

## 最佳实践

### 1. 命名约定

- **命名空间**: 使用机器人ID或功能模块，如 `robot1`, `nav_system`
- **话题名称**: 保持简洁明了，如 `cmd`, `status`, `sensor_data`

### 2. QoS选择

- **default**: 控制指令、配置更新（可靠传输）
- **sensor_data**: 传感器数据、状态信息（最佳努力）
- **parameter_events**: 系统事件、参数变化（可靠传输）
- **system_default**: 一般数据传输（最佳努力）

### 3. 队列大小

- **小队列 (1-10)**: 实时控制，低延迟需求
- **中等队列 (10-50)**: 一般数据传输
- **大队列 (50+)**: 批量数据、事件日志

### 4. 错误处理

始终检查节点输出的 `messageReceived` 端口来确认是否成功接收到消息。

## 故障排除

### 常见问题

1. **话题不存在**: 检查发布者是否运行，话题名称是否正确
2. **权限问题**: 确保ROS2环境正确配置
3. **QoS不匹配**: 发布者和订阅者的QoS设置必须兼容
4. **命名空间错误**: 检查完整话题路径是否正确

### 验证步骤

```bash
# 1. 检查话题是否存在
ros2 topic list

# 2. 检查话题类型
ros2 topic info /your/topic/name

# 3. 检查是否有数据
ros2 topic echo /your/topic/name

# 4. 检查QoS设置
ros2 topic info /your/topic/name --verbose
```

### 调试提示

- **context**: 通常保持默认值0，除非有特殊需求
- **nodeNamespace**: 空字符串表示全局命名空间
- **qosProfile**: 使用预定义的配置文件名称
- **queueSize**: 根据数据频率和重要性调整
- **topicName**: 确保与发布者使用相同的话题名称

---

*简化的输入端口配置使得ROS2订阅节点更易使用和维护，同时保持了必要的灵活性。*
