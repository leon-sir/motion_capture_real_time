# VRPN Gravity Projection

ROS节点：从VRPN位姿数据计算重力投影向量，用于机器人姿态估计和控制。

## 功能

- 订阅VRPN位姿话题，提取四元数姿态
- 将世界坐标系重力向量投影到机器人本体坐标系
- 实时发布重力投影向量
- 支持变化检测，姿态变化时打印日志
- 自动检测VRPN连接状态

## 安装

```bash
cd ~/vrpn_ws
catkin_make
source devel/setup.bash
```

## 使用

```bash
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch
```

这会自动启动 vrpn_client_ros 连接动捕服务器（默认 172.16.21.168）。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server` | `172.16.21.168` | VRPN服务器IP地址 |
| `port` | `3883` | VRPN服务器端口 |
| `rigid_body` | `Rigid` | 刚体名称 |
| `vrpn_topic` | `/vrpn_client_node/Rigid/pose` | VRPN位姿输入话题 |
| `gravity_topic` | `/projected_gravity` | 重力投影输出话题 |
| `gravity_x/y/z` | `0, 0, -9.81` | 世界坐标系重力向量 |
| `change_threshold` | `0.1` | 变化检测阈值 |

### 示例

```bash
# 连接不同的动捕服务器
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch server:=192.168.1.100

# 指定刚体名称
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch rigid_body:=Robot1

# 调整变化检测阈值
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch change_threshold:=0.05
```

## 话题

### 订阅
- `/vrpn_client_node/Rigid/pose` (geometry_msgs/PoseStamped)

### 发布
```
/projected_gravity (geometry_msgs/Vector3Stamped)

rostopic echo /projected_gravity
rostopic echo /projected_velocity
```

## 原理

重力投影计算公式：
```
projected_gravity = q^(-1) * gravity_world
```

其中 `q` 是机器人姿态四元数，`gravity_world` 是世界坐标系重力向量。

### 姿态与重力投影关系

| 机器人姿态 | 重力投影 (x, y, z) |
|-----------|-------------------|
| 直立 | (0, 0, -9.81) |
| 前倾 | (+, 0, -) |
| 后仰 | (-, 0, -) |
| 左倾 | (0, -, -) |
| 右倾 | (0, +, -) |
| 水平旋转 | (0, 0, -9.81) 不变 |

## 连接检测

节点会自动检测VRPN数据：
- 启动后3秒无数据：警告 "No VRPN data received!"
- 运行中数据中断：警告 "VRPN data timeout"

## 依赖

- ROS Noetic
- Eigen3
- geometry_msgs
- vrpn_client_ros
