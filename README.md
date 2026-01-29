# VRPN Gravity Projection

ROS节点：从VRPN位姿数据计算重力投影向量，用于机器人姿态估计和控制。

## 功能

- 订阅VRPN位姿话题，提取四元数姿态
- 将世界坐标系重力向量投影到机器人本体坐标系
- 实时发布重力投影向量
- 支持变化检测，姿态变化时打印日志

## 安装

```bash
cd ~/vrpn_ws
catkin_make
source devel/setup.bash
```

## 使用

```bash
ping 172.16.21.168
roslaunch vrpn_client_ros sample.launch server:=172.16.21.168

rostopic echo /vrpn_client_node/Rigid/pose

###求逆
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch enable_velocity:=true enable_ang_velocity:=true velocity_source:=twist ang_velocity_source:=twist debug:=true

##转换四元素
roslaunch vrpn_gravity_projection reset_four_element.launch


转换twist(x,y,z)-->(x,-z,y)
rosrun vrpn_gravity_projection twist_echo
rosrun vrpn_gravity_projection accel_echo

rostopic echo /projected_gravity

###四元素计算角速度 20hz
rosrun vrpn_gravity_projection velocity_node 


# 方法1：使用launch文件
roslaunch vrpn_gravity_projection twist_transform.launch

# 方法2：直接运行节点
rosrun vrpn_gravity_projection twist_transform

# 查看转换后的twist
rostopic echo /twist_body

```

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vrpn_topic` | `/vrpn_client_node/Rigid/pose` | VRPN位姿输入话题 |
| `gravity_topic` | `/projected_gravity` | 重力投影输出话题 |
| `gravity_x/y/z` | `0, 0, -9.81` | 世界坐标系重力向量 |
| `change_threshold` | `0.1` | 变化检测阈值 |
| `debug` | `false` | 调试输出开关 |

### 示例

```bash
# 自定义VRPN话题
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch vrpn_topic:=/vrpn_client_node/Robot/pose

# 启用调试输出
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch _debug:=true

# 调整变化检测阈值
roslaunch vrpn_gravity_projection vrpn_gravity_projection.launch _change_threshold:=0.05
```

## 话题

### 订阅
- `/vrpn_client_node/Rigid/pose` (geometry_msgs/PoseStamped)

### 发布
- `/projected_gravity` (geometry_msgs/Vector3Stamped)

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

## 依赖

- ROS Noetic
- Eigen3
- geometry_msgs
- vrpn_client_ros
