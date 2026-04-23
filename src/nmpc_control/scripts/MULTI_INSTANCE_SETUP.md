# 多实例控制器启动指南

## 🎯 目标

为并行自动调参启动多个控制器实例，每个实例连接到对应的仿真UAV。

---

## 📋 快速启动步骤

### Windows: 启动仿真实例

```batch
# 运行 windows/rflysim/SITLRun.bat
Please input UAV swarm number: 4

# 等待所有实例启动完成:
# - 4个 CopterSim
# - 4个 PX4 SITL
# - QGroundControl 显示 4 个 UAV
```

### Linux/WSL: 启动控制器

#### 方法1: 一键启动脚本（推荐）

```bash
cd /root/trajectory_project
source devel/setup.bash

# 启动4个NMPC控制器实例
./src/nmpc_control/scripts/start_multi_nmpc.sh 4
```

**输出示例**:
```
===========================================================
启动 4 个NMPC控制器实例
===========================================================

实例配置:
-----------------------------------------------------------
  实例 1 (UAV 1):
    - ROS命名空间: /uav1
    - MAVLink端口: 14540
    - 里程计话题: /uav1/vio/odometry
    - 控制话题: /uav1/mavros/setpoint_raw/attitude
  实例 2 (UAV 2):
    - ROS命名空间: /uav2
    - MAVLink端口: 14550
    ...
```

#### 方法2: 手动逐个启动

```bash
# 终端1: UAV 1
roslaunch nmpc_control rflysim_nmpc_multi.launch uav_id:=1

# 终端2: UAV 2
roslaunch nmpc_control rflysim_nmpc_multi.launch uav_id:=2

# 终端3: UAV 3
roslaunch nmpc_control rflysim_nmpc_multi.launch uav_id:=3

# 终端4: UAV 4
roslaunch nmpc_control rflysim_nmpc_multi.launch uav_id:=4
```

---

## 🔍 验证启动

### 检查ROS话题

```bash
rostopic list | grep uav
```

**预期输出**:
```
/uav1/vio/odometry
/uav1/mavros/state
/uav1/mavros/setpoint_raw/attitude
/uav1/nmpc_control/update_weights
/uav2/vio/odometry
/uav2/mavros/state
...
```

### 检查MAVROS连接

```bash
# UAV 1
rostopic echo /uav1/mavros/state -n 1

# UAV 2
rostopic echo /uav2/mavros/state -n 1
```

预期看到 `connected: True`

### 检查里程计数据

```bash
# UAV 1
rostopic echo /uav1/vio/odometry -n 1

# UAV 2
rostopic echo /uav2/vio/odometry -n 1
```

预期看到位置、速度数据

---

## 📊 数据流架构

```
┌─────────────────────────────────────────────────┐
│           Windows (仿真环境)                     │
│  ┌───────────┐  ┌───────────┐                   │
│  │CopterSim 1│  │CopterSim 2│  ...              │
│  │Port 14540 │  │Port 14550 │                   │
│  └─────┬─────┘  └─────┬─────┘                   │
│        │              │                          │
│  ┌─────▼─────┐  ┌─────▼─────┐                   │
│  │ PX4 SITL 1│  │ PX4 SITL 2│  ...              │
│  └─────┬─────┘  └─────┬─────┘                   │
└────────┼──────────────┼──────────────────────────┘
         │ UDP          │ UDP
         │ 14540        │ 14550
         ▼              ▼
┌─────────────────────────────────────────────────┐
│         Linux/WSL (ROS环境)                      │
│                                                  │
│  Namespace: /uav1        Namespace: /uav2       │
│  ┌───────────────┐      ┌───────────────┐       │
│  │   MAVROS 1    │      │   MAVROS 2    │       │
│  │ Port: 14540   │      │ Port: 14550   │       │
│  └───┬───────┬───┘      └───┬───────┬───┘       │
│      │       │              │       │            │
│      │       │              │       │            │
│  ┌───▼───┐ ┌▼──────┐   ┌───▼───┐ ┌▼──────┐     │
│  │ Odom  │ │ NMPC  │   │ Odom  │ │ NMPC  │     │
│  │ Node  │ │Control│   │ Node  │ │Control│     │
│  └───────┘ └───────┘   └───────┘ └───────┘     │
│                                                  │
└──────────────────────────────────────────────────┘
```

---

## 🛠️ 配置说明

### 端口映射规则

根据`windows/rflysim/SITLRun.bat`脚本：
```
UAV ID  →  MAVLink端口
  1     →  14540
  2     →  14550  (14540 + 10)
  3     →  14560  (14540 + 20)
  4     →  14570  (14540 + 30)
  ...
  N     →  14540 + (N-1)×10
```

Launch文件自动计算：
```xml
<arg name="mavlink_port" default="$(eval 14540 + (arg('uav_id') - 1) * 10)" />
```

### ROS命名空间

每个实例独立命名空间：
- UAV 1: `/uav1/*`
- UAV 2: `/uav2/*`
- UAV 3: `/uav3/*`
- UAV 4: `/uav4/*`

### 话题列表

每个实例的话题（以UAV 1为例）：

| 话题 | 说明 |
|------|------|
| `/uav1/vio/odometry` | 里程计数据（来自RflySim） |
| `/uav1/mavros/state` | 飞控状态 |
| `/uav1/mavros/local_position/pose` | 本地位置 |
| `/uav1/mavros/setpoint_raw/attitude` | 姿态控制指令 |
| `/uav1/nmpc_control/update_weights` | 参数更新话题 |
| `/uav1/cmd/trajectory` | 轨迹指令 |

---

## 🐛 故障排查

### 问题1: MAVROS无法连接

**症状**: `rostopic echo /uav1/mavros/state` 显示 `connected: False`

**检查**:
```bash
# 检查端口是否监听
netstat -ano | grep 14540  # Windows
ss -tuln | grep 14540      # Linux
```

**解决**:
- 确认对应PX4 SITL已启动
- 检查端口号是否正确匹配
- Windows防火墙可能阻止UDP通信

### 问题2: 无里程计数据

**症状**: `/uav1/vio/odometry` 无数据

**检查**:
```bash
# 检查节点是否运行
rosnode list | grep uav1

# 应该看到:
# /uav1/rflysim_odom_node
# /uav1/nmpc_control_node
```

**解决**:
- 检查CopterSim是否正常运行
- 确认`copter_id`参数正确
- 查看节点日志: `rosnode info /uav1/rflysim_odom_node`

### 问题3: 话题命名空间错误

**症状**: 自动调参找不到话题

**原因**: 话题使用了绝对路径而非相对路径

**解决**: 
Launch文件中使用相对路径：
```xml
<!-- ✓ 正确 -->
<param name="odom_topic" value="vio/odometry" />

<!-- ✗ 错误 -->
<param name="odom_topic" value="/vio/odometry" />
```

---

## 📝 日志查看

### 脚本启动的日志

```bash
# 实时查看UAV 1日志
tail -f /tmp/nmpc_uav1.log

# 查看所有实例日志
tail -f /tmp/nmpc_uav*.log
```

### ROS节点日志

```bash
# 查看所有节点
rosnode list

# 查看特定节点日志
rosrun rqt_console rqt_console

# 或使用roslog
roslog list
```

---

## 🚀 启动并行调参

控制器启动完成后：

```bash
python3 src/benchmark_utils/scripts/auto_tuner_parallel.py \
    --controller nmpc \
    --workers 4 \
    --trials 20
```

---

## 🛑 停止控制器

### 方法1: 停止所有实例

```bash
pkill -f 'rflysim_nmpc_multi.launch'
```

### 方法2: 逐个停止

按 Ctrl+C 在每个launch终端

---

## ✅ 检查清单

启动并行调参前，确认：

- [ ] Windows端：`windows/rflysim/SITLRun.bat` 启动了N个仿真实例
- [ ] QGroundControl 显示N个UAV连接
- [ ] Linux端：启动了N个控制器实例
- [ ] `rostopic list | grep uav` 显示所有话题
- [ ] 每个`/uavX/mavros/state` 显示 `connected: True`
- [ ] 每个`/uavX/vio/odometry` 有数据输出
- [ ] 所有UAV处于待命状态（未起飞）

一切就绪！🎯
