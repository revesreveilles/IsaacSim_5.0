# Isaac Sim 5.0 到 4.5 控制器迁移指南

## 📋 概述

本文档详细说明如何将在 Isaac Sim 5.0 中创建的自定义控制器节点迁移到 Isaac Sim 4.5 环境中。

## 🎯 迁移背景

Isaac Sim 不同版本之间存在扩展路径和结构差异：

- **开发环境**: Isaac Sim 5.0 安装目录中的扩展文件夹
- **运行环境**: Isaac Sim 4.5 用户数据目录中的扩展缓存

## 📂 路径结构对比

### Isaac Sim 5.0 开发路径
```
/home/user/isaac-sim/isaac-sim-standalone@5.x.x/exts/isaacsim.robot.wheeled_robots/
```

### Isaac Sim 4.5 运行时路径
```
/home/user/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/isaacsim.robot.wheeled_robots-4.0.4+106.5.0.lx64.r.cp310/
```

## 🚀 迁移步骤

### 步骤 1: 确认目标路径

首先确认 Isaac Sim 4.5 的实际扩展路径：

```bash
# 查找 Isaac Sim 4.5 扩展目录
find ~/.local/share/ov/data -name "isaacsim.robot.wheeled_robots*" -type d
```

典型路径格式：
```
~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/isaacsim.robot.wheeled_robots-[版本号]/
```

### 步骤 2: 文件迁移

#### 2.1 核心控制器文件
将以下文件从源目录复制到目标目录：

```
源: isaac-sim/isaacsim-5.0/source/extensions/isaacsim.robot.wheeled_robots/
目标: ~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/isaacsim.robot.wheeled_robots-*/

需要复制的文件:
├── isaacsim/robot/wheeled_robots/controllers/
│   ├── your_controller.py                    # 您的控制器实现
│   └── __init__.py                          # 包含新控制器的导入
├── isaacsim/robot/wheeled_robots/nodes/
│   ├── OgnYourController.ogn                # 节点定义文件
│   └── OgnYourController.py                 # 节点实现文件
└── isaacsim/robot/wheeled_robots/ogn/
    └── OgnYourControllerDatabase.py         # 数据库文件
```

#### 2.2 创建 Python 节点副本
Isaac Sim 4.5 需要在 `ogn/python/nodes/` 目录中也有节点文件的副本：

```
isaacsim/robot/wheeled_robots/ogn/python/nodes/
├── OgnYourController.ogn
└── OgnYourController.py
```

#### 2.3 创建 USD 模板文件
在 `ogn/tests/usd/` 目录中创建模板文件：

```
isaacsim/robot/wheeled_robots/ogn/tests/usd/
└── OgnYourControllerTemplate.usda
```

### 步骤 3: 关键配置检查

#### 3.1 ⚠️ 重要：分类设置一致性

**这是迁移中最容易出错的地方！** 必须确保 `.ogn` 文件和 `Database.py` 文件中的分类设置完全一致。

**在 `.ogn` 文件中：**
```json
{
    "YourController": {
        "categories": {
            "isaacSim": "您的控制器描述"
        }
    }
}
```

**在 `Database.py` 文件中必须对应：**
```python
node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, 
                      "isaacSim,您的控制器描述")
```

**错误示例：**
```python
# ❌ 错误：分类名称不匹配
node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacWheeledRobots")  # 与 .ogn 不一致
```

#### 3.2 节点注册代码

确保在节点 Python 文件末尾添加注册代码：

```python
# 在 OgnYourController.py 文件末尾
OgnYourControllerDatabase.register(OgnYourController)
```

#### 3.3 控制器导入

检查 `controllers/__init__.py` 文件是否包含您的控制器导入：

```python
from .your_controller import YourController
```

## 🔧 常见问题和解决方案

### ❌ 问题 1: 节点未出现在 Isaac Sim 4.5 中

**可能原因：**
- 文件未复制到正确的运行时路径
- 分类设置不一致
- 节点注册代码缺失

**解决方案：**
1. 确认所有文件都复制到了 `~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/isaacsim.robot.wheeled_robots-*/` 
2. 重启 Isaac Sim 以重新加载扩展
3. 检查扩展管理器中 `isaacsim.robot.wheeled_robots` 是否已启用

### ❌ 问题 2: 分类设置错误

**常见错误：** 节点出现在错误的分类下或无法找到

**原因：** `.ogn` 文件和 `Database.py` 文件中的分类设置不匹配

**正确设置示例：**

**.ogn 文件：**
```json
"categories": {
    "isaacSim": "Four Wheel Drive Four Wheel Steering Controller"
}
```

**Database.py 文件：**
```python
node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, 
                      "isaacSim,Four Wheel Drive Four Wheel Steering Controller")
```

### ❌ 问题 3: 导入错误

**常见错误**: `No module named 'carb'` 或其他模块导入失败

**说明**: 这是正常现象，以下模块只在 Isaac Sim 运行时环境中可用：
- `carb`
- `omni.graph.core`
- `isaacsim.core.*`


### ❌ 问题 4: 文件路径结构不正确

**Isaac Sim 4.5 要求的完整文件结构：**

```
isaacsim.robot.wheeled_robots-*/
├── isaacsim/robot/wheeled_robots/
│   ├── controllers/
│   │   ├── your_controller.py           # 控制器实现
│   │   └── __init__.py                 # 包含导入
│   ├── nodes/
│   │   ├── OgnYourController.ogn       # 节点定义
│   │   └── OgnYourController.py        # 节点实现
│   └── ogn/
│       ├── OgnYourControllerDatabase.py # 数据库文件
│       ├── python/nodes/                # Python 节点副本
│       │   ├── OgnYourController.ogn
│       │   └── OgnYourController.py
│       └── tests/usd/                   # USD 模板
│           └── OgnYourControllerTemplate.usda
```

## 📝 版本兼容性注意事项

### API 差异检查清单

1. **导入路径验证**
   - 确保所有 `from isaacsim.*` 导入在 4.5 中有效
   - 检查是否使用了 5.0 特有的 API

2. **依赖关系检查**
   - 验证控制器依赖的其他模块是否在 4.5 中可用
   - 确认第三方库版本兼容性

3. **接口变更适配**
   - 某些 API 参数或返回值可能有变化
   - 检查控制器基类 `BaseController` 的接口是否一致

### 配置文件注意事项

1. **extension.toml**
   - 通常不需要修改
   - 版本号会由系统管理

2. **依赖版本**
   - 扩展依赖会自动适配到 4.5 版本
   - 无需手动修改依赖版本号

## ✅ 迁移验证清单

完成迁移后，请检查以下项目：

- [ ] **文件复制完成**
  - [ ] 控制器实现文件已复制
  - [ ] `controllers/__init__.py` 已更新
  - [ ] OGN 节点定义文件已复制
  - [ ] 数据库文件已复制
  - [ ] Python 节点副本已创建
  - [ ] USD 模板文件已创建

- [ ] **配置正确**
  - [ ] 分类设置一致（`.ogn` 和 `Database.py`）
  - [ ] 节点注册代码已添加
  - [ ] 控制器导入已添加到 `__init__.py`

- [ ] **清理和测试**
  - [ ] Python 缓存已清理
  - [ ] Isaac Sim 已重启
  - [ ] 扩展状态已确认
  - [ ] 节点在 OmniGraph 中可见

## 🎯 总结

通过遵循本指南，您可以成功将 Isaac Sim 5.0 中开发的自定义控制器迁移到 Isaac Sim 4.5 环境中。关键是理解两个版本之间的路径差异，并确保所有必要的文件都复制到了正确的位置。

迁移完成后，您的自定义控制器应该能够在 Isaac Sim 4.5 的 OmniGraph 编辑器中正常使用。
