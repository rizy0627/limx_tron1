# Tron1 ROS2 Control Package

`tron1_ctrl_ros2` 是针对 Tron1 双足/轮足机器人的 ROS2 控制包。它基于 WebSocket SDK 封装，提供了标准的 ROS2 接口来控制机器人的运动、切换状态、设置参数以及获取传感器数据。

## 功能特性

*   **WebSocket 通信**：自动连接机器人，支持断线重连和自动发现序列号 (ACCID)。
*   **运动控制**：通过 `/cmd_vel` 话题控制机器人移动（支持全向移动）。
*   **状态反馈**：实时发布 IMU (`/imu`)、里程计 (`/odom`) 和机器人本体状态 (`/robot_status`)。
*   **模式切换**：提供完整的服务接口控制站立、行走、蹲下、恢复、急停等。
*   **高级功能**：支持设置身高、灯效、楼梯模式、原地踏步模式等。

## 依赖项

在编译之前，请确保已安装以下系统依赖：

```bash
sudo apt-get update
sudo apt-get install -y libboost-all-dev libwebsocketpp-dev nlohmann-json3-dev
```

ROS2 依赖项（通常随 ROS2 安装）：
*   `rclcpp`
*   `geometry_msgs`
*   `sensor_msgs`
*   `nav_msgs`
*   `std_msgs`
*   `std_srvs`

## 编译

在您的 ROS2 工作空间根目录下（例如 `~/tron1_ws`）：

```bash
cd ~/tron1_ws
colcon build --packages-select tron1_ctrl_ros2
source install/setup.bash
```

## 运行

### 使用 Launch 文件（推荐）

Launch 文件预置了默认参数：

```bash
ros2 launch tron1_ctrl_ros2 tron1.launch.py
```

### 使用 ros2 run

```bash
ros2 run tron1_ctrl_ros2 tron1_node --ros-args -p server_uri:="ws://10.192.1.2:5000"
```

## 参数说明

可以在 Launch 文件中修改这些参数：

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `server_uri` | string | `ws://10.192.1.2:5000` | 机器人的 WebSocket 服务地址 |
| `accid` | string | `""` (空) | 机器人序列号。留空则自动从首条消息中发现 |
| `max_linear_x` | double | `1.0` | 对应 SDK x=1.0 的最大线速度 (m/s) |
| `max_linear_y` | double | `1.0` | 对应 SDK y=1.0 的最大横移速度 (m/s) |
| `max_angular_z` | double | `1.0` | 对应 SDK z=1.0 的最大角速度 (rad/s) |

## 接口文档

### 1. 话题 (Topics)

#### 订阅 (Subscribers)
*   **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
    *   控制机器人移动。
    *   `linear.x`: 前进/后退
    *   `linear.y`: 左右横移
    *   `angular.z`: 旋转

#### 发布 (Publishers)
*   **`/imu`** (`sensor_msgs/msg/Imu`)
    *   发布机器人的 IMU 数据（四元数、角速度、线加速度）。
*   **`/odom`** (`nav_msgs/msg/Odometry`)
    *   发布机器人的里程计数据（仅在轮足模式下有效）。
*   **`/robot_status`** (`tron1_ctrl_ros2/msg/RobotStatus`)
    *   发布机器人的详细状态信息。

### 2. 服务 (Services)

所有服务均为**同步调用**，即会等待机器人确认执行结果后才返回。

#### 基础控制 (std_srvs/Trigger)
*   **`/stand_mode`**: 切换到站立模式。
*   **`/walk_mode`**: 切换到行走模式。
*   **`/sit_down`**: 蹲下。
*   **`/recover`**: 摔倒后自动恢复/爬起。
*   **`/emergency_stop`**: 紧急停止（切断电机输出）。

#### 功能开关 (std_srvs/SetBool)
*   **`/stair_mode`**: 开启/关闭楼梯模式（仅轮足）。
*   **`/marktime_mode`**: 开启/关闭原地踏步模式（仅双足）。
*   **`/enable_odom`**: 开启/关闭里程计推送。
*   **`/enable_imu`**: 开启/关闭 IMU 推送。

#### 高级控制 (自定义服务)
*   **`/set_base_height`** (`tron1_ctrl_ros2/srv/SetBaseHeight`)
    *   调整机器人身高。
    *   `direction`: `1` (升高), `-1` (降低)。
*   **`/set_light_effect`** (`tron1_ctrl_ros2/srv/SetLightEffect`)
    *   设置灯光效果。
    *   `effect_id`: 1-21 (详见 `.srv` 文件定义或下方列表)。

### 3. 自定义消息定义

#### `msg/RobotStatus.msg`
```
std_msgs/Header header
string accid          # 序列号
string sw_version     # 软件版本
string imu_status     # IMU 健康状态
string camera_status  # 相机健康状态
string motor_status   # 电机健康状态
int8 battery          # 电池电量 (0-100)
string status         # 当前运行状态 (WALK, STAND, SIT, ERROR...)
```

#### `srv/SetBaseHeight.srv`
```
int8 direction # 1: up, -1: down
---
bool success
string message
```

#### `srv/SetLightEffect.srv`
```
# 常量定义 (部分示例)
uint8 STATIC_RED = 1
uint8 STATIC_GREEN = 2
...
uint8 FAST_FLASH_WHITE = 21

uint8 effect_id
---
bool success
string message
```

## 使用示例

**控制灯光为红色常亮：**
```bash
ros2 service call /set_light_effect tron1_ctrl_ros2/srv/SetLightEffect "{effect_id: 1}"
```

**让机器人站起来：**
```bash
ros2 service call /stand_mode std_srvs/srv/Trigger
```

**查看机器人状态：**
```bash
ros2 topic echo /robot_status
```
