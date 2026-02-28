# RQT UAV QGC Plugin (`rqt_uav_qgc`)

这是一个基于 ROS 2 和 `rqt` 框架开发的无人机地面站（Ground Control Station）管控插件。主要用于实现对多台无人机的远程并行控制（如启动 `MicroXRCEAgent`）、终端状态交互，以及实时的 ROS 2 话题数据监控。

## ✨ 主要功能与特性

- **多无人机并行管理**：支持通过界面输入 IP、用户和密码，批量管理多台无人机设备。
- **内建远程 SSH 终端**：基于纯 Python `paramiko` 实现持久化远程控制，支持读取并渲染带有 ANSI 颜色的终端日志，脱离密码输入弹窗依赖。
- **一键并行操作**：支持 **"Start All Drones"** 命令，一键自动按序完成连接、启动节点等业务链路。
- **高自由度扩展**：基于 Suffix（后缀）绑定的自动发现框架。未来需要增加或减少无人机数量、监控数量时，只需在 Qt Designer 修改 `.ui` 文件，代码端即可**免修改自动适配**。
- **ROS 2 动态话题监控**：
  - **自动推导数据类型**：只需要输入话题名称即可，插件会自动查询 ROS 2 图并探测所需的数据格式（支持任意第三方与自定义 Message 类型）。
  - **频率与数据监测**：支持实时显示 Hz 统计，及摘要字段内容预览。

---

## 🛠️ 依赖项

在使用本仓库之前，请确保系统中已经安装了以下依赖：

- ROS 2 (如 Humble)
- ROS 2 RQT 相关依赖
- Python SSH 库 `paramiko`

```bash
# 安装 Python 依赖（必需）
pip install paramiko

# 安装必要的 ROS 2 RQT 依赖(通常安装ROS2会配套安装)
sudo apt update
sudo apt install ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-gui ros-$ROS_DISTRO-rqt-gui-py
```

---

## 🚀 编译与安装

1. 确保将本仓库 clone 到了个人 ROS 2 工作空间的 `src` 目录下。
2. 回到工作空间根目录进行编译：

```bash
cd ~/uavProject/ws_rcir_qgc    # 替换为你的工作空间路径
colcon build --packages-select rqt_uav_qgc --symlink-install
```

---

## 🏃 运行指南

在运行该插件之前，一定要 **Source 所需要的工作空间**。

### 🚨 避坑指南：自定义消息无法识别（如 px4_msgs）
如果你的话题使用自定义消息格式（例如 `/fmu/out/vehicle_status` 对应 `px4_msgs/msg/VehicleStatus`），即便通过命令行可以查询到该消息，RQT 的 Python 进程也需要通过 `PYTHONPATH` 来加载它。
因此，**你必须在启动插件的当前终端里**，提前加载包含该 msg 的工作空间：

```bash
# 先加载自定义消息空间（例如 px4_msgs 或 uav_msgs 所在的空间）
source /path/to/your/custom_msg_ws/install/setup.bash

# 再加载当前 rqt 插件空间
source ~/uavProject/ws_rcir_qgc/install/setup.bash
```

### 启动插件
推荐使用附带的 `launch` 文件启动。这可以确保 RQT 强制清空旧的缓存并在独立的 Node 进行发现，从而避免遇到 UI 更新失败的异常：

```bash
ros2 launch rqt_uav_qgc rqt_uav_qgc.launch.py
# 等价于以下命令
ros2 run rqt_uav_qgc rqt_uav_qgc --force-discover --clear-config
```

取消`--clear-config`将会启动上次退出时的rqt缓存，加载上次退出时的配置。

---

## 🧩 如何修改与扩展界面 (UI)

如果你需要添加第 4 台、第 5 台无人机的面板，或者增加第二个话题监控区域，完全**不需要**编写任何 Python 代码。

1. 进入 `src/rqt_uav_qgc/resource/` 目录。
2. 运行 `designer UavQgc.ui` 打开界面编辑器。
3. 复制对应的模块：
   - 复制卡片 `droneCard` 到合适位置，并将新的容器名称修改为 `droneCard_4`。容器内的子控件后缀需保持一致（会自动根据外衣名称推导）。
   - 复制监控区 `monitorFrame`，新容器更名为 `monitorFrame_2`。
4. 重新 `colcon build` 后运行启动命令，代码就会自动挂载所有回调与逻辑。

