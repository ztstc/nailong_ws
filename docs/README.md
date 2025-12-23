# ROS 2 Humble 迁移 - 入门指南

欢迎！本指南将帮助您将这个项目从ROS 1迁移到ROS 2 Humble。

## 快速开始

### 1. 当前状态
- ✅ 已创建 `ros2-humble` 分支
- ✅ 已编写完整的迁移文档
- ⏳ 待做：迁移代码

### 2. 可用资源

本项目在 `/home/zhengtuo/nailong_ws/docs/` 中包含以下文档：

#### 📚 学习资料
- **[ROS1_vs_ROS2_Guide.md](./ROS1_vs_ROS2_Guide.md)** - 深度对比指南（推荐初学者先读）
  - 核心架构差异
  - 详细的API变化
  - 编程示例对比
  - 常见问题解答

- **[QUICK_REFERENCE.md](./QUICK_REFERENCE.md)** - 快速参考表
  - API映射表
  - 消息类型映射
  - 常见陷阱和解决方案

#### 🔧 实操指南
- **[CODE_MIGRATION_EXAMPLES.md](./CODE_MIGRATION_EXAMPLES.md)** - 代码迁移示例
  - 节点初始化
  - Pub/Sub迁移
  - Launch文件转换
  - CMakeLists.txt迁移
  - Package.xml迁移

#### 📋 项目计划
- **[MIGRATION_PLAN.md](./MIGRATION_PLAN.md)** - 项目迁移总体计划
  - 项目结构分析
  - 分阶段迁移步骤
  - 关键迁移点

- **[MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md)** - 详细检查清单
  - 按包列出的检查项目
  - 测试要点
  - 常见问题排查

#### 🛠️ 辅助工具
- **[ros2_migrate_helper.sh](./ros2_migrate_helper.sh)** - 自动化迁移脚本
  ```bash
  bash docs/ros2_migrate_helper.sh help
  ```

## 建议的学习路径

### 第一阶段：理解差异（1-2天）
1. 阅读 [ROS1_vs_ROS2_Guide.md](./ROS1_vs_ROS2_Guide.md) 的前3个章节
2. 查看 [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) 的API映射表
3. 使用辅助脚本检查项目中的ROS1代码：
   ```bash
   bash docs/ros2_migrate_helper.sh check-ros1-headers
   bash docs/ros2_migrate_helper.sh check-nodhandle
   bash docs/ros2_migrate_helper.sh check-rosmacros
   ```

### 第二阶段：代码迁移（3-7天）
1. 阅读 [CODE_MIGRATION_EXAMPLES.md](./CODE_MIGRATION_EXAMPLES.md)
2. 按照 [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md) 逐个迁移包
3. 从简单的包开始（如 open_cv）
4. 使用快速参考表处理具体问题

### 第三阶段：测试和验证（1-2天）
1. 编译整个项目
2. 运行每个节点和launch文件
3. 检查所有功能是否正常

## 关键要点速览

### 三大主要变化

#### 1️⃣ 节点初始化
```cpp
// ROS1
ros::NodeHandle nh;

// ROS2
auto node = std::make_shared<MyNode>();
class MyNode : public rclcpp::Node { ... }
```

#### 2️⃣ 日志和参数
```cpp
// ROS1
ROS_INFO("msg");
nh.getParam("param", value);

// ROS2
RCLCPP_INFO(this->get_logger(), "msg");
this->declare_parameter("param", default_value);
auto value = this->get_parameter("param").as_double();
```

#### 3️⃣ Launch文件
```bash
# ROS1: .launch (XML)
# ROS2: .launch.py (Python)
```

## 项目包介绍

| 包名 | 描述 | 优先级 | 复杂度 |
|------|------|--------|--------|
| open_cv | 图像处理 | 🔴 高 | ⭐ 低 |
| inertial_nav | 惯性导航 | 🔴 高 | ⭐⭐ 中 |
| ros_uart_protocol | UART通信 | 🟡 中 | ⭐⭐ 中 |
| navi_demo01 | 导航演示 | 🟡 中 | ⭐⭐⭐ 高 |
| urdf01, urdf02 | URDF模型 | 🟢 低 | ⭐ 低 |

**建议迁移顺序**: open_cv → inertial_nav → ros_uart_protocol → urdf01/urdf02 → navi_demo01

## 实用命令参考

### 检查和备份
```bash
# 进入项目目录
cd /home/zhengtuo/nailong_ws

# 检查当前分支
git branch -v

# 查看ROS1头文件使用
bash docs/ros2_migrate_helper.sh check-ros1-headers

# 备份重要文件
bash docs/ros2_migrate_helper.sh backup-launch
bash docs/ros2_migrate_helper.sh backup-cmake
bash docs/ros2_migrate_helper.sh backup-package
```

### 编译
```bash
# 清理旧编译
bash docs/ros2_migrate_helper.sh clean-build

# 编译项目
colcon build --symlink-install

# 查看编译错误
colcon build --event-handlers console_direct+
```

### 测试
```bash
# 激活环境
source install/setup.bash

# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 运行节点
ros2 run package_name executable_name

# 启动launch文件
ros2 launch package_name launch_file.launch.py
```

## 常见问题

### Q: 我应该从哪个包开始？
**A**: 从 `open_cv` 开始，因为它是最简单的，没有复杂的依赖关系。这样可以在迁移简单代码时积累经验。

### Q: 如何处理ROS导航栈的迁移？
**A**: ROS导航栈在ROS2中已经更新为 Nav2。您可能需要参考官方的 Nav2 文档来适配 `navi_demo01`。

### Q: 可以同时保留ROS1和ROS2版本吗？
**A**: 可以。main 分支保留ROS1版本，ros2-humble 分支用于ROS2版本。

### Q: 迁移后需要修改硬件相关的代码吗？
**A**: 一般不需要，除非涉及某些ROS特定的接口。大多数硬件抽象层代码保持不变。

## 获取帮助

- 📖 查看文档中的详细说明
- 🔍 使用辅助脚本检查具体问题
- 💬 参考 [ROS Discourse](https://discourse.ros.org/)
- 🐛 查看 [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) 的常见陷阱部分

## 下一步

1. **立即**: 读完本文件和 [ROS1_vs_ROS2_Guide.md](./ROS1_vs_ROS2_Guide.md)
2. **今天**: 运行迁移检查脚本了解项目规模
3. **本周**: 开始迁移第一个包（open_cv）
4. **本月**: 完成所有包的迁移和测试

---

**祝您迁移顺利！** 🚀

如有任何问题，请参考相应的文档或在 [ROS Discourse](https://discourse.ros.org/) 社区求助。

