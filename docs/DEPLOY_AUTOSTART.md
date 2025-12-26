# ROS2 开机自启部署指南（底盘 + LD06，无RViz）

本指南帮助你在 Linux 上为 ROS2 项目配置开机自启服务，分别启动底盘驱动（`base_driver`）与 LD06 激光雷达（`ldlidar_stl_ros2`）。

## 前置条件
- 已安装 ROS2 Humble（或与你系统相匹配的发行版）。
- 工作区：`/home/zhengtuo/nailong_ros2_ws` 已可构建。
- 设备串口：底盘一般为 `/dev/ttyACM0`，LD06 一般为 `/dev/ttyUSB0`，实际值以你的设备为准。

## 一次性准备
```bash
# 进入工作区并构建
cd /home/zhengtuo/nailong_ros2_ws
source /opt/ros/humble/setup.bash
colcon build

# 使脚本可执行
chmod +x scripts/start_chassis.sh
chmod +x scripts/start_ld06.sh

# 手工测试（建议先在前台验证）
source install/setup.bash
ros2 launch base_driver base_driver.launch.py serial_port:=/dev/ttyACM0 baudrate:=115200
# 新开终端，再测试LD06（默认端口 /dev/ttyUSB0）
ros2 launch ldlidar_stl_ros2 ld06.launch.py
```

如需修改 LD06 串口名或参数，可编辑 `scripts/start_ld06.sh` 中的 `PORT_NAME_OVERRIDE` 或在执行时增加 launch 参数：
```bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py port_name:=/dev/ttyUSB0
```

## 添加 udev 规则（可选但推荐）
为 LD06 和底盘创建稳定符号链接并放宽权限，避免串口名波动：

1. 识别设备：
```bash
lsusb
# 记录激光雷达的 idVendor 和 idProduct
```
2. 创建规则文件（示例，需按实际 VID/PID 调整）：
```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ld06", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ld06.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
之后端口可使用 `/dev/ld06`。将 `scripts/start_ld06.sh` 中的 `PORT_NAME_OVERRIDE="/dev/ld06"` 即可。

底盘如需规则，方法类似，替换为底盘串口的 VID/PID。

## 安装 systemd 服务
将服务文件复制到系统目录并启用开机启动：
```bash
sudo cp /home/zhengtuo/nailong_ros2_ws/systemd/ros2-chassis.service /etc/systemd/system/
sudo cp /home/zhengtuo/nailong_ros2_ws/systemd/ros2-ld06.service /etc/systemd/system/

# 如需变更运行用户，请编辑这两个 .service 文件中的 User/Group 字段

sudo systemctl daemon-reload
sudo systemctl enable ros2-chassis.service
sudo systemctl enable ros2-ld06.service

# 立即启动并查看日志
sudo systemctl start ros2-chassis.service
sudo systemctl start ros2-ld06.service
journalctl -u ros2-chassis.service -f
journalctl -u ros2-ld06.service -f
```

## 验证
```bash
# 查看节点
ros2 node list
# 查看话题（底盘与雷达）
ros2 topic list
# 查看激光雷达数据
ros2 topic echo /scan --qos-profile sensor_data
```

## 故障排查
- 权限问题：确认串口设备权限或添加 udev 规则。
- 环境未加载：确保脚本中 `source /opt/ros/<distro>/setup.bash` 与 `source <ws>/install/setup.bash` 路径正确。
- 串口占用：确认无其他进程占用 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。
- 重启策略：`Restart=always` 会自动重启；可通过 `journalctl` 查看错误原因。

## 可选：合并到一个服务
你也可创建一个联合启动脚本，在同一服务中用 `tmux` 或 `supervisord` 管理两个进程；当前方案分服务更直观、可独立控制与排错。
