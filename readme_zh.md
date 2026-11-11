
# 介绍
## Unitree mujoco
`unitree_mujoco` 是 Unitree 机器人的 MuJoCo 仿真器。C++ 仿真器现在是标准 ROS 2 包，直接发布与真机一致的 `unitree_go` 或 `unitree_hg` topic；Python 仿真器仍使用 `unitree_sdk2_python`。

![](./doc/func.png)

## 目录结构
- `simulate`: 基于 ROS 2 和 MuJoCo 的 C++ 仿真器（推荐）
- `simulate_python`: 基于 unitree_sdk2py 和 mujoco (python) 实现的仿真器
- `unitree_robots`: unitree_sdk2 支持的机器人 mjcf 描述文件
- `terrain_tool`: 仿真场景地形生成工具
- `example`: 例程

## 支持的 ROS 2 接口
**当前版本仅支持底层开发，主要用于控制器的 sim to real 验证**
- `/lowcmd`：电机控制指令
- `/lowstate`：电机状态
- `/sportmodestate`：机器人位置和速度
- `/wirelesscontroller`：仿真 Unitree 手柄状态
- `/secondary_imu`：胸部 IMU 数据（仅 G1）
- `/lf/bmsstate`：电池状态（仅 G1）

## 消息(DDS idl)类型说明
- Unitree Go2, B2, H1, B2w, Go2w 型号的机器人使用 unitree_go idl 实现底层通信
- Unitree G1, H1-2 型号的机器人使用 unitree_hg 实现底层通信

注：
 1. 电机的编号与机器人实物一致，具体可参考 [Unitree 文档](https://support.unitree.com/home/zh/developer)
 2. 在机器人实物上关闭自带的运控服务后， `SportModeState` 消息是无法读取的。仿真中保留了这一消息，便于用户利用位置和速度信息分析所开发的控制程序。

## 相关链接
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree 文档](https://support.unitree.com/home/zh/developer)
- [mujoco doc](https://mujoco.readthedocs.io/en/stable/overview.html)

# 安装
## c++ 仿真器 (simulate)
### 1. 依赖

```bash
sudo apt install libyaml-cpp-dev libboost-all-dev libglfw3-dev \
  ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros ros-jazzy-rmw-cyclonedds-cpp
```

工作区或 underlay 中必须提供 `unitree_ros2` 的 `unitree_go` 和 `unitree_hg` 消息包。

#### mujoco

下载 MuJoCo [4.0.0 安装包](https://github.com/google-deepmind/mujoco/releases/tag/4.0.0)，解压到 `~/.mujoco` 目录下。

```
cd unitree_mujoco/simulate/
ln -sfnT ~/.mujoco/mujoco-4.0.0 mujoco
```

### 2. 编译 unitree_mujoco
```
cd /path/to/unitree_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_mujoco
source install/setup.bash
```

### 3. 测试:
运行：
```bash
source src/unitree_lowlevel/scripts/setup.sh lo jazzy
ros2 run unitree_mujoco unitree_mujoco -r go2 -s scene_terrain.xml
```
可以看到加载了 Go2 机器人的 mujoco 仿真器。

## Python 仿真器 (simulate_python)
### 1. 依赖
#### unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
如果遇到问题：
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
参考: https://github.com/unitreerobotics/unitree_sdk2_python

#### mujoco-python
```bash
pip3 install mujoco
```
#### joystick
```bash
pip3 install pygame
```
### 2. 测试
```bash
cd ./simulate_python
python3 ./unitree_mujoco.py
```
在新终端运行
```bash
python3 ./test/test_unitree_sdk2.py
```
程序会输出机器人在仿真器中的姿态和位置信息，同时机器人的每个电机都会持续输出 1Nm 的转矩。

**注：** 测试程序发送的是 unitree_go 消息，如果需要测试 G1 机器人，需要修改程序使用 unitree_hg 消息。


 

# 使用
## 1. 仿真配置
### c++ 仿真器
c++ 仿真器的配置文件位于 `/simulate/config.yaml` 中：

使用 `--config /path/to/config_trampoline.yaml` 可加载另一份完整 YAML，替代安装目录的
`config/config.yaml`。配置文件的相对路径按当前工作目录解析，显式命令行参数覆盖所选 YAML。
启用深度相机时，所选配置文件的目录或其上一级目录必须包含 `camera_profiles.yaml`。禁用时只需
`depth_camera: {enabled: false}`，不读取或校验其余相机字段及相机配置目录文件。
`--depth-camera` 在加载相机配置前覆盖 YAML 中的开关。`eval` 的相对路径也按所选配置的目录解析。

`episode_services: true` 可在没有仿真内监控器时启用 `/unitree_mujoco/reset_episode`、
`start_episode` 和 `episode_status`。Reset 恢复模型默认状态，start 开始回合计时；
任务结果由客户端判定。

`episode_monitor: <name>` 选择仿真内任务监控器；内置只有 `beam`，留空表示不监控。
默认 `config.yaml` 使用 `eval: ""`，因此普通启动既没有任务监控器，也不提供 reset 服务。

任务选择与机器人配置分离：`--config` 选择机器人（默认 `config.yaml`，Go2 用
`config/go2.yaml`，后者包含硬件模型和相机标定），`--eval` 应用任务覆盖。推荐蹦床
部署基线是 `--config simulate/config/go2.yaml --eval simulate/config/eval_trampoline.yaml`：
`scene_trampoline.xml` + `go2.xml`，2 ms、PGS、implicitfast；模型质量 16.514412 kg，源码同一行记录的
硬件质量是 16.45 kg（`unitree_robots/go2/go2.xml:67`），两者不可与更轻的训练资产混用。
```yaml
# 仿真器加载的机器人名称
# "go2", "b2", "b2w", "h1"
robot: "go2"

# 机器人仿真仿真场景文件
# 以 go2 为例，指的是/unitree_robots/go2/文件夹下的 scene.xml 文件
robot_scene: "scene.xml" 

# 是否输出机器人连杆、关节、传感器等信息，1为输出
print_scene_information: 1

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 H1 机器人初始化挂起的过程 
enable_elastic_band: 0 # For H1 
```

### ROS 2 深度相机

C++ 仿真器可以发布与 `realsense-ros` 相同的校正后深度接口：

- `/camera/depth/image_rect_raw`：`sensor_msgs/msg/Image`，`16UC1`
- `/camera/depth/camera_info`：`sensor_msgs/msg/CameraInfo`
- `/tf_static`：`base_link -> camera_link -> camera_depth_frame -> camera_depth_optical_frame`

在 `simulate/config.yaml` 中设置 Go2 相机安装位姿以及 ROS frame/topic，在
`simulate/camera_profiles.yaml` 中选择分辨率、帧率、深度单位和内参。`mount`
表示 `camera_link` 相对 `base_link` 的位姿。设置 `exclude_self: true` 可隐藏 Go2 的视觉几何；
碰撞几何始终不会出现在深度图中。

深度图退化由 `depth_camera.noise` 内联配置，因此每个任务覆盖文件可以携带与训练分布
匹配的噪声模型。该块使用明确的 `process_order`，支持 `stereo_occlusion`、
`correlated_dropout` 和 `pixel_dropout`。省略 `depth_camera.noise` 时发布无噪声的
渲染深度图。

<!-- https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf P68 Table 3-52 -->
启用深度相机后，主视窗会在配置的 mount 位姿中心显示一个无碰撞的
`90 x 25 x 25 mm` D435I 参考 box。该 box 不会出现在传感器图像中，也不会把
datasheet 中标称的 `75 g` 质量加入 Go2 动力学。

```bash
cd /home/rkz/code/unitree_ws
source src/unitree_lowlevel/scripts/setup.sh lo jazzy
ros2 run unitree_mujoco unitree_mujoco \
  --config simulate/config/go2.yaml \
  --eval simulate/config/eval_beam.yaml
```

默认 `config.yaml` 使用 `eval: ""`，不加载任务配置。Beam 任务用
`--config simulate/config/go2.yaml --eval simulate/config/eval_beam.yaml` 启动。
YAML 里的 `eval` 字段按该文件所在目录解析；命令行 `--eval` 按当前目录解析，
也支持绝对路径。`eval_beam.yaml` 集中设置 `robot_scene`、`episode_monitor`、
`episode_services`、`inertial_gt`、`episode_timeout_s`、`max_payload_count`、
`payload_mode`、`payload_seed`，以及该任务的 `depth_camera.enabled` 和噪声模型。
相机标定（外参、型号 profile、frame、topic）留在机器人配置 `config/go2.yaml`。
显式命令行参数覆盖 YAML，布尔选项支持 `=false`。修改源码中的配置后需重新构建安装。

`max_payload_count: N` 在加载 XML 模型时从 `0..N` 均匀采样 payload 数量，
并使用与 MJLab Beam payload 配置相同的 `1 kg`、`0.15 x 0.08 x 0.03 m`
方块和篮筐惯量生成无悬空的随机堆叠。方块不含 joint，相对 `base_link`
固定不动。网格为 2×3 列，每列最多 6 层，与训练一致；参数范围为 `0..36`。
篮筐使用与训练完全相同的开口长方体网格，尺寸为 0.30×0.24×0.15 m。

`payload_mode: fixed` 保持启动时的数量和布局；`resample` 在每次 `ResetEpisode`
时重新采样，需要 `episode_monitor: beam`，相邻样本允许相同。`payload_seed` 控制
独立的随机序列：启动消耗第一个样本，之后每次重新采样的 reset 消耗下一个。
`max_payload_count: 0` 保留 0.76 kg 空篮子；省略数量并选择 `fixed` 才会禁用
篮子和负载。每轮负载详情随 episode id 写入仿真日志。视窗自带的状态 reset
按钮不触发负载重新采样。

`episode_timeout_s` 从 `StartEpisode` 开始按仿真时间计时，到期且尚无任务结果
时报告 `TIMEOUT`，后续 reset 由评测脚本执行。

对于 Go2 模型，仿真器会以 50 Hz 在 `/unitree_go2/inertial_delta` 发布合成后的
base-link 惯量参数增量，消息类型为 `std_msgs/msg/Float32MultiArray`。10 个值的
Log-Cholesky 顺序和 nominal 模型差分与 MJLab Beam GT observation 完全一致。
重新采样时会同步更新动力学、可见负载和惯性真值。

机器人 bridge 和相机共用一个 ROS 2 context。`ROS_DOMAIN_ID`、RMW 和网卡只由
source 后的环境决定；以上命令使两者都在 domain 0、`lo` 网卡上使用 CycloneDDS。
仿真器只发布深度流，不会伪造 D435I 的彩色、红外或 IMU 数据。

### python 仿真器
python 仿真器的配置文件位于 `/simulate_python/config.py` 中：
```python
# 仿真器加载的机器人名称
# "go2", "b2", "b2w", "h1"
ROBOT = "go2" 

# 机器人仿真仿真场景文件
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene

# dds domain id，最好与实物(实物上默认为 0)区分开
DOMAIN_ID = 1 # Domain id
# 网卡名称, 对于仿真建议使用本地回环 "lo"
INTERFACE = "lo" # Interface 

# 是否输出机器人连杆、关节、传感器等信息，True 为输出
PRINT_SCENE_INFORMATION = True 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 H1 机器人初始化挂起的过程 
ENABLE_ELASTIC_BAND = False 

# 仿真步长 单位(s)
# 为保证仿真的可靠性，需要大于 viewer.sync() 渲染一次所需要的时间
SIMULATE_DT = 0.003  

# 可视化界面的运行步长，0.02 对应 50fps/s
VIEWER_DT = 0.02 
```

### 游戏手柄
仿真器会使用 Xbox 或 Switch 手柄模拟机器人的无线控制器，C++ 仿真器将其发布到 `/wirelesscontroller`。没有手柄时，需要将 `config.yaml/config.py` 中的 `use_joystick/USE_JOYSTICK` 设置为 0。如果手柄不属于 Xbox 或 Switch 映射，可以在源码中修改映射（可使用 `jstest` 查看按键和摇杆 id）：

在 `simulate/src/physics_joystick.h` 中：
```C++
 if (js_type == "xbox")
{
    js_id_.axis["LX"] = 0; // Left stick axis x
    js_id_.axis["LY"] = 1; // Left stick axis y
    js_id_.axis["RX"] = 3; // Right stick axis x
    js_id_.axis["RY"] = 4; // Right stick axis y
    js_id_.axis["LT"] = 2; // Left trigger
    js_id_.axis["RT"] = 5; // Right trigger
    js_id_.axis["DX"] = 6; // Directional pad x
    js_id_.axis["DY"] = 7; // Directional pad y
    
    js_id_.button["X"] = 2;
    js_id_.button["Y"] = 3;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 6;
    js_id_.button["START"] = 7;
}
```

In `simulate_python/unitree_sdk2_bridge.py`: 
```python
if js_type == "xbox":
    self.axis_id = {
        "LX": 0,  # Left stick axis x
        "LY": 1,  # Left stick axis y
        "RX": 3,  # Right stick axis x
        "RY": 4,  # Right stick axis y
        "LT": 2,  # Left trigger
        "RT": 5,  # Right trigger
        "DX": 6,  # Directional pad x
        "DY": 7,  # Directional pad y
    }

    self.button_id = {
        "X": 2,
        "Y": 3,
        "B": 1,
        "A": 0,
        "LB": 4,
        "RB": 5,
        "SELECT": 6,
        "START": 7,
    }
```
### 人形机器人虚拟挂带
考虑到人形机器人不便于从平地上启动并进行调试，在仿真中设计了一个虚拟挂带，用于模拟人形机器人的吊起和放下。设置 `enable_elastic_band/ENABLE_ELASTIC_BAND = 1` 可以启用虚拟挂带。加载机器人后，按 `9` 启用或松开挂带，按 `7` 放下机器人，按 `8` 吊起机器人。

## 2. 地形生成工具
我们提供了一个在 mujoco 仿真器中参数化创建简单地形的工具，支持添加楼梯、杂乱地面、高程图等地形。程序位于 `terrain_tool` 文件夹中。具体的使用方法见 `terrain_tool` 文件夹下的 readme 文件。
![](./doc/terrain.png)

## 3. sim to real
`example` 文件夹下提供了使用不同接口实现 Go2 机器人站起再趴下的简单例子。这些例子简演示了如何使用 Unitree 提供的接口实现仿真到实物的实现。下面是每个文件夹名称的解释：
- `cpp`: 基于 `C++`, 使用 `unitree_sdk2` 接口
- `python`: 基于 `python`，使用 `unitree_sdk2_python` 接口
- `ros2`: 基于`ros2`，使用 `unitree_ros2` 接口

### unitree_sdk2 
1. 编译运行
```bash
cd example/cpp
mkdir build && cd build
cmake ..
make -j4
```
运行：
```bash
./stand_go2 # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
./stand_go2 enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```

2. sim to real
```C++
if (argc < 2)
{   
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactory::Instance()->Init(1, "lo");
}
else
{   
    // 否则使用指定的网卡
    ChannelFactory::Instance()->Init(0, argv[1]);
}
```

### unitree_sdk2_python
1. 运行：
```bash
python3 ./stand_go2.py # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
python3 ./stand_go2.py enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```
2. sim to real

```python
if len(sys.argv) <2:
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactoryInitialize(1, "lo")
else:
    // 否则使用指定的网卡
    ChannelFactoryInitialize(0, sys.argv[1])
```
### unitree_ros2

1. 编译安装
首先确保已经正确配置好 unitree_ros2 环境，见 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)。
```bash
source ~/unitree_ros2/setup.sh
cd example/ros2
colcon build
```

2. 运行仿真
```bash
source ~/unitree_ros2/setup_local.sh # 使用本地网卡
export ROS_DOMAIN_ID=1 # 修改domain id 与仿真一致
./install/stand_go2/bin/stand_go2 # 运行
```

3. 运行实物
```bash
source ~/unitree_ros2/setup.sh # 使用机器人连接的网卡
export ROS_DOMAIN_ID=0 # 使用默认的 domain id 
./install/stand_go2/bin/stand_go2 # 运行
```
