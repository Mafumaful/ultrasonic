# Ultrasonic Costmap Layer

Nav2 costmap 插件，用于将超声波传感器数据转换为弧形障碍物，适用于 ROS2 Navigation2 导航系统。

## 概述

本软件包提供了一个 ROS2 Navigation2 (Nav2) costmap 层插件，订阅超声波传感器数据并在局部代价地图中生成基于概率模型的弧形障碍物。该插件专为 `car_chassis/msg/Ultrasonic` 消息类型设计，支持三个超声波传感器（左、中、右）的读数。

## 核心特性

- **概率传感器模型**: 基于 RangeSensorLayer 的概率模型，提供更真实的障碍物表示
- **弧形障碍物生成**: 根据超声波传感器特性创建扇形/弧形障碍物区域
- **三传感器支持**: 独立处理左、中、右三个超声波传感器，支持不同的安装角度和位置偏移
- **射线清除功能**: 可选的射线清除算法，清除传感器检测范围内的自由空间
- **传感器位置偏移**: 支持配置每个传感器在 base_link 坐标系中的位置偏移（x, y 平移）
- **完全可配置**: 所有传感器参数（角度、范围、FOV、概率阈值等）均可通过 YAML 配置
- **Nav2 兼容**: 完全集成到 Nav2 导航栈，作为标准 costmap 层使用

## 代码结构

### 文件组织

```
ultrasonic_costmap_layer/
├── CMakeLists.txt                          # CMake 构建配置
├── package.xml                             # ROS2 包元数据和依赖
├── README.md                               # 本文档
├── TESTING.md                              # 测试指南（中文）
├── ultrasonic_layer.xml                    # Pluginlib 插件描述文件
├── include/
│   └── ultrasonic_costmap_layer/
│       └── ultrasonic_layer.hpp            # UltrasonicLayer 类头文件
├── src/
│   └── ultrasonic_layer.cpp                # UltrasonicLayer 实现
├── config/
│   └── ultrasonic_layer_params.yaml        # 示例配置参数
└── test/
    ├── test_ultrasonic_costmap.launch.py   # 测试用 Launch 文件
    └── verify_plugin.sh                    # 插件验证脚本
```

### 主要类和组件

#### UltrasonicLayer (ultrasonic_layer.hpp/cpp)

主类，继承自 `nav2_costmap_2d::CostmapLayer`，实现超声波传感器数据到代价地图的转换。

**关键方法：**

| 方法 | 功能 | 位置 |
|------|------|------|
| `onInitialize()` | 初始化插件，加载参数，创建订阅器 | src/ultrasonic_layer.cpp:57 |
| `updateBounds()` | 更新需要修改的代价地图区域边界 | src/ultrasonic_layer.cpp:448 |
| `updateCosts()` | 将层的代价值合并到主代价地图中 | src/ultrasonic_layer.cpp:484 |
| `bufferIncomingUltrasonicMsg()` | 接收并缓冲超声波消息 | src/ultrasonic_layer.cpp:223 |
| `updateCostmap()` | 处理缓冲的消息，更新内部代价地图 | src/ultrasonic_layer.cpp:236 |
| `processSingleSensor()` | 处理单个传感器的读数 | src/ultrasonic_layer.cpp:252 |
| `updateCostmapWithSensor()` | 使用传感器模型更新代价地图单元格 | src/ultrasonic_layer.cpp:295 |
| `update_cell()` | 使用贝叶斯更新单个单元格的概率 | src/ultrasonic_layer.cpp:411 |

**传感器模型方法：**

| 方法 | 功能 | 位置 |
|------|------|------|
| `sensor_model()` | 计算给定位置的障碍物概率 | src/ultrasonic_layer.cpp:203 |
| `gamma()` | 角度衰减函数（FOV 内的概率权重） | src/ultrasonic_layer.cpp:177 |
| `delta()` | 距离衰减函数（径向距离的概率权重） | src/ultrasonic_layer.cpp:186 |
| `get_deltas()` | 计算射线追踪的步长 | src/ultrasonic_layer.cpp:191 |

### 核心算法

#### 1. 概率传感器模型

基于论文的概率传感器模型，考虑以下因素：

- **角度衰减 γ(θ)**: 在传感器 FOV 边缘，检测概率降低
- **距离衰减 δ(φ)**: 使用 tanh 函数在障碍物距离附近创建平滑过渡
- **贝叶斯更新**: 结合先验概率和传感器测量，计算后验概率

```
P(occupied | sensor) = (sensor_prob * prior) / (sensor_prob * prior + (1 - sensor_prob) * (1 - prior))
```

#### 2. 扇形区域标记

对于每个传感器读数：

1. 计算传感器在全局坐标系中的位置（考虑位置偏移和机器人姿态）
2. 根据传感器角度和 FOV 确定扇形区域的边界
3. 使用 orient2d (叉积) 判断每个单元格是否在扇形区域内
4. 对扇形内的单元格应用概率传感器模型

#### 3. 射线清除

可选功能，清除传感器到障碍物之间的自由空间：

1. 从传感器原点沿传感器方向发射射线
2. 清除距离在 `[0, distance - ray_clear_margin]` 范围内的单元格
3. 使用 Bresenham 风格的射线追踪算法

### 参数系统

所有参数在 `onInitialize()` 中通过 ROS2 参数服务器声明和获取：

```cpp
declareParameter("sensor_angle_left", rclcpp::ParameterValue(45.0));
node->get_parameter(name_ + "." + "sensor_angle_left", angle_left_deg);
```

## 依赖项

### 系统依赖

- **ROS2 Humble** 或更高版本
- **C++17** 编译器

### ROS2 包依赖

| 包名 | 用途 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `nav2_costmap_2d` | Nav2 代价地图框架 |
| `pluginlib` | 插件加载机制 |
| `car_chassis` | 包含 `Ultrasonic.msg` 定义 |
| `tf2` / `tf2_ros` | 坐标变换 |
| `angles` | 角度归一化工具 |

## 编译安装

```bash
# 在工作空间根目录
cd ~/Cyber_dog_mini  # 或你的工作空间

# 编译本包
colcon build --packages-select ultrasonic_costmap_layer

# 加载环境
source install/setup.bash

# 验证安装
ros2 pkg prefix ultrasonic_costmap_layer
```

## 使用方法

### 集成到 Nav2 配置

将超声波层添加到 Nav2 costmap 配置文件中：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Costmap 基础配置
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05

      # 插件列表
      plugins: ["ultrasonic_layer", "inflation_layer"]

      # 超声波层配置
      ultrasonic_layer:
        plugin: "ultrasonic_costmap_layer/UltrasonicLayer"
        enabled: true
        ultrasonic_topic: "/ultrasonic"

        # 传感器角度（度）
        sensor_angle_left: 45.0
        sensor_angle_mid: 0.0
        sensor_angle_right: -45.0

        # 传感器位置偏移（米，在 base_link 坐标系）
        sensor_left_tx: 0.0     # 左传感器 x 偏移
        sensor_left_ty: 0.1     # 左传感器 y 偏移
        sensor_mid_tx: 0.15     # 中传感器 x 偏移
        sensor_mid_ty: 0.0      # 中传感器 y 偏移
        sensor_right_tx: 0.0    # 右传感器 x 偏移
        sensor_right_ty: -0.1   # 右传感器 y 偏移

        # 量程限制（毫米）
        min_range: 50.0
        max_range: 4000.0
        distance_scale: 1.0

        # 传感器视场角（度）
        sensor_fov: 30.0

        # 概率模型参数
        phi: 1.2
        inflate_cone: 1.0
        clear_threshold: 0.2
        mark_threshold: 0.8

        # 射线清除参数
        enable_ray_clear: true
        ray_clear_margin: 0.10
        ray_clear_radius_cells: 0

        # 其他参数
        no_readings_timeout: 2.0
        clear_on_max_reading: false

      # 膨胀层配置
      inflation_layer:
        plugin: "nav2_costmap_2d/InflationLayer"
        enabled: true
        cost_scaling_factor: 3.0
        inflation_radius: 0.3
```

### 测试插件

独立测试超声波 costmap 层：

```bash
# 终端 1: 启动测试 costmap 节点
ros2 launch ultrasonic_costmap_layer test_ultrasonic_costmap.launch.py

# 终端 2: 发布测试超声波数据
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 1000, mid: 1500, right: 800}" --rate 10

# 终端 3: 可视化（RViz）
rviz2
```

在 RViz 中：
1. 设置 Fixed Frame 为 `odom`
2. 添加 `Map` 显示
3. 设置 Topic 为 `/ultrasonic_costmap`
4. 你应该能看到弧形障碍物

## 配置参数详解

所有参数均在初始化时从 ROS2 参数服务器加载（参见 src/ultrasonic_layer.cpp:76-152）。

### 基础配置

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `enabled` | bool | `true` | 启用/禁用本层 | cpp:77 |
| `ultrasonic_topic` | string | `/ultrasonic` | 订阅的超声波话题 | cpp:80 |

### 传感器角度配置

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `sensor_angle_left` | double | `45.0` | 左传感器角度（度，逆时针为正） | cpp:86 |
| `sensor_angle_mid` | double | `0.0` | 中传感器角度（度） | cpp:88 |
| `sensor_angle_right` | double | `-45.0` | 右传感器角度（度） | cpp:89 |

### 传感器位置偏移配置

这些参数定义每个传感器在 base_link 坐标系中的安装位置（米）：

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `sensor_left_tx` | double | `0.0` | 左传感器 x 轴偏移（米） | cpp:147 |
| `sensor_left_ty` | double | `0.0` | 左传感器 y 轴偏移（米） | cpp:148 |
| `sensor_mid_tx` | double | `0.0` | 中传感器 x 轴偏移（米） | cpp:149 |
| `sensor_mid_ty` | double | `0.0` | 中传感器 y 轴偏移（米） | cpp:150 |
| `sensor_right_tx` | double | `0.0` | 右传感器 x 轴偏移（米） | cpp:151 |
| `sensor_right_ty` | double | `0.0` | 右传感器 y 轴偏移（米） | cpp:152 |

### 量程配置

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `min_range` | double | `50.0` | 最小有效量程（毫米） | cpp:98 |
| `max_range` | double | `4000.0` | 最大有效量程（毫米） | cpp:99 |
| `distance_scale` | double | `1.0` | 距离缩放因子：真实距离(mm) = 传感器值 × distance_scale | cpp:104 |

### 传感器视场角配置

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `sensor_fov` | double | `30.0` | 每个传感器的视场角（度） | cpp:108 |

### 概率模型参数

这些参数控制传感器概率模型的行为：

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `phi` | double | `1.2` | 距离衰减函数的拐点参数（控制障碍物区域的锐度） | cpp:115 |
| `inflate_cone` | double | `1.0` | 扇形区域膨胀系数（>1 扩大扇形，<1 缩小） | cpp:118 |
| `clear_threshold` | double | `0.2` | 清除阈值（概率 < 此值标记为自由空间） | cpp:121 |
| `mark_threshold` | double | `0.8` | 标记阈值（概率 > 此值标记为障碍物） | cpp:124 |
| `clear_on_max_reading` | bool | `false` | 最大量程读数时是否清除整个射线 | cpp:127 |

### 射线清除参数

控制从传感器到障碍物之间的自由空间清除：

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `enable_ray_clear` | bool | `true` | 启用射线清除功能 | cpp:131 |
| `ray_clear_margin` | double | `0.10` | 清除到障碍物前的安全边距（米） | cpp:134 |
| `ray_clear_radius_cells` | int | `0` | 射线清除半径（单元格数，0=单像素射线） | cpp:137 |

### 其他参数

| 参数名 | 类型 | 默认值 | 说明 | 代码位置 |
|--------|------|--------|------|----------|
| `no_readings_timeout` | double | `2.0` | 传感器数据超时时间（秒） | cpp:112 |
| `transform_tolerance` | double | `0.1` | TF 变换容差（秒） | cpp:155 |

## 工作原理

### 数据流程

```
超声波硬件
    ↓
car_chassis/msg/Ultrasonic 消息
    ↓
bufferIncomingUltrasonicMsg()  (cpp:223) - 缓冲消息
    ↓
updateCostmap()  (cpp:236) - 批量处理
    ↓
processSingleSensor()  (cpp:252) - 逐个传感器处理
    ↓
updateCostmapWithSensor()  (cpp:295) - 应用传感器模型
    ↓
update_cell()  (cpp:411) - 贝叶斯更新单元格
    ↓
updateCosts()  (cpp:484) - 合并到主 costmap
```

### 扇形障碍物生成算法

对每个有效的超声波传感器读数执行以下步骤（参见 updateCostmapWithSensor，cpp:295）：

**1. 计算传感器全局位置**
```cpp
// 从 base_link 坐标系转换到全局坐标系（cpp:284-287）
const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
const double sensor_ox = robot_x + c * tx_b - s * ty_b;
const double sensor_oy = robot_y + s * tx_b + c * ty_b;
```

**2. 定义扇形边界**
- 传感器朝向：`robot_yaw + sensor_angle`
- FOV 范围：`±sensor_fov / 2`
- 障碍物距离：从传感器读数计算（考虑 distance_scale）

**3. 判断单元格是否在扇形内**
使用 orient2d（叉积）判断点是否在三角形内（cpp:392-396）：
```cpp
int w0 = orient2d(Ax, Ay, Bx, By, x, y);
int w1 = orient2d(Bx, By, Ox, Oy, x, y);
int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);
update_xy_cell = (w0 >= thr) && (w1 >= thr) && (w2 >= thr);
```

**4. 应用概率模型**
对扇形内的每个单元格调用 `update_cell()`

### 概率传感器模型

实现基于论文的概率模型（参见 sensor_model，cpp:203）：

**角度衰减 γ(θ)**（cpp:177-184）：
```
γ(θ) = 1 - (θ / max_angle)²    if |θ| ≤ max_angle
γ(θ) = 0                        otherwise
```
在 FOV 边缘，检测置信度降低。

**距离衰减 δ(φ)**（cpp:186-189）：
```
δ(φ) = 1 - (1 + tanh(2(φ - phi_v))) / 2
```
使用 tanh 函数在障碍物距离附近创建平滑过渡。

**综合传感器模型**（cpp:203-219）：
```
λ = δ(φ) · γ(θ)

if φ < r - 2Δr:           P = (1 - λ) · 0.5
elif φ < r - Δr:          P = λ · 0.5 · ((φ - (r - 2Δr)) / Δr)² + (1 - λ) · 0.5
elif φ < r + Δr:          P = λ · ((1 - 0.5 · ((r - φ) / Δr)²) - 0.5) + 0.5
else:                     P = 0.5
```

**贝叶斯更新**（cpp:428-439）：
```
P(occupied | sensor) = (sensor_prob × prior) /
                       (sensor_prob × prior + (1 - sensor_prob) × (1 - prior))
```

### 射线清除算法

当 `enable_ray_clear = true` 时（cpp:324-340）：

1. 从传感器原点沿传感器方向发射射线
2. 使用 get_deltas() 计算射线步长（cpp:191）
3. 清除距离在 `[0, distance - ray_clear_margin]` 范围内的单元格
4. 每个单元格设置为 `clear = true`，强制概率为 0

这确保传感器到障碍物之间的空间被正确标记为自由。

### 坐标系

- **base_link**: 机器人本体坐标系（前进方向为 x 轴正方向）
- **odom / map**: 全局坐标系（通常是 odom 用于局部代价地图）
- **传感器角度**: 相对于 base_link 的 x 轴，逆时针为正
- **传感器位置**: 在 base_link 中的 (tx, ty) 偏移

## 调试和调优

### 参数调优指南

#### 1. 调整传感器位置偏移

如果障碍物位置不准确，需要校准传感器安装位置：

```yaml
# 测量每个传感器在 base_link 中的实际位置
sensor_left_tx: 0.12    # 左传感器在车前方 12cm
sensor_left_ty: 0.08    # 左传感器在车左侧 8cm
sensor_mid_tx: 0.15     # 中传感器在车最前方 15cm
sensor_mid_ty: 0.0      # 中传感器在车中线上
sensor_right_tx: 0.12
sensor_right_ty: -0.08  # 右传感器在车右侧 8cm（负值）
```

#### 2. 调整概率阈值

**障碍物过于敏感**（误报太多）：
```yaml
mark_threshold: 0.9     # 提高标记阈值（默认 0.8）
clear_threshold: 0.15   # 降低清除阈值（默认 0.2）
```

**障碍物不够敏感**（漏报太多）：
```yaml
mark_threshold: 0.7     # 降低标记阈值
clear_threshold: 0.25   # 提高清除阈值
```

#### 3. 调整扇形宽度

**扇形过宽**：
```yaml
sensor_fov: 25.0        # 减小 FOV（默认 30.0）
inflate_cone: 0.8       # 缩小扇形区域（默认 1.0）
```

**扇形过窄**：
```yaml
sensor_fov: 35.0        # 增大 FOV
inflate_cone: 1.2       # 扩大扇形区域
```

#### 4. 调整障碍物锐度

**障碍物边界太模糊**：
```yaml
phi: 0.8                # 减小 phi 使过渡更锐利（默认 1.2）
```

**障碍物边界太锐利**（可能导致不连续）：
```yaml
phi: 1.5                # 增大 phi 使过渡更平滑
```

#### 5. 调整射线清除

**清除过于激进**（清除了不该清除的区域）：
```yaml
ray_clear_margin: 0.15  # 增大安全边距（默认 0.10）
enable_ray_clear: false # 或完全禁用射线清除
```

**清除不够**（障碍物消失后残留）：
```yaml
ray_clear_margin: 0.05  # 减小安全边距
clear_on_max_reading: true  # 最大量程时也清除
```

### 常见问题排查

#### 问题 1: 没有看到障碍物

**检查步骤：**

1. 验证数据发布：
```bash
ros2 topic echo /ultrasonic
# 检查 left, mid, right 值是否在 [min_range, max_range] 内
```

2. 检查插件是否加载：
```bash
ros2 param list | grep ultrasonic_layer
# 应该看到所有 ultrasonic_layer 参数
```

3. 检查日志输出：
```bash
# 查看节点日志，应该看到初始化信息（cpp:165-172）
# [INFO] UltrasonicLayer: Initialized and subscribed to /ultrasonic
# [INFO] Sensor angles (deg): L=45.0, M=0.0, R=-45.0
```

4. 检查读数有效性：
```bash
# 确保读数在有效范围内
ros2 topic echo /ultrasonic --field left
# 值应该在 50-4000 之间（或你的配置范围）
```

#### 问题 2: 障碍物位置不正确

**可能原因：**

1. **distance_scale 不正确**：
   - 测量实际距离并与传感器读数对比
   - 调整 `distance_scale` 参数

2. **传感器角度错误**：
   - 验证 `sensor_angle_left/mid/right` 与硬件安装匹配
   - 逆时针为正，顺时针为负

3. **传感器位置偏移未配置**：
   - 测量并配置 `sensor_*_tx` 和 `sensor_*_ty` 参数

4. **TF 变换问题**：
```bash
# 检查 TF 树
ros2 run tf2_tools view_frames
# 确保 base_link -> odom 变换正确
```

#### 问题 3: 插件未加载

**检查步骤：**

1. 验证插件描述文件：
```bash
cat install/ultrasonic_costmap_layer/share/ultrasonic_costmap_layer/ultrasonic_layer.xml
# 应该包含插件类定义
```

2. 检查 pluginlib 注册：
```bash
ros2 pkg prefix ultrasonic_costmap_layer
# 应该输出安装路径
```

3. 重新编译：
```bash
colcon build --packages-select ultrasonic_costmap_layer --event-handlers console_direct+
```

#### 问题 4: 性能问题

**症状：** costmap 更新缓慢，导航延迟

**解决方案：**

1. 减小更新区域：
```yaml
sensor_fov: 25.0        # 减小 FOV
```

2. 降低 costmap 分辨率：
```yaml
resolution: 0.1         # 从 0.05 增大到 0.1（更粗糙但更快）
```

3. 禁用射线清除（如果不需要）：
```yaml
enable_ray_clear: false
```

#### 问题 5: 与其他层冲突

**症状：** ultrasonic_layer 的障碍物被其他层覆盖或清除

**解决方案：**

在 `updateCosts()` (cpp:529-538) 中实现了安全的层合并逻辑：
- 仅在 `NO_INFORMATION` 或 `FREE_SPACE` 时清除
- 仅当新代价更高时标记障碍物

如果仍有问题，调整插件顺序：
```yaml
plugins: [
  "static_layer",
  "ultrasonic_layer",    # 在 obstacle_layer 之前
  "obstacle_layer",
  "inflation_layer"
]
```

## 测试示例

### 使用测试数据

发布测试数据以在 RViz 中查看弧形障碍物：

```bash
# 示例 1: 前方 1.5m 有障碍物
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 2000, mid: 1500, right: 2000}" --rate 10

# 示例 2: 左侧 1m 有障碍物
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 1000, mid: 3000, right: 3000}" --rate 10

# 示例 3: 三个方向都有不同距离的障碍物
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 800, mid: 1500, right: 1200}" --rate 10
```

### 完整测试流程

参见 TESTING.md 文件获取详细的测试指南。

快速测试：
```bash
# 1. 启动测试节点
ros2 launch ultrasonic_costmap_layer test_ultrasonic_costmap.launch.py

# 2. 发布测试数据
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 1000, mid: 1500, right: 800}" --rate 10

# 3. 查看 costmap 更新
ros2 topic echo /ultrasonic_costmap_updates
```

## 与 Nav2 集成

### 完整的 Nav2 配置示例

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Costmap 基础配置
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05

      # 插件列表（顺序很重要）
      plugins: [
        "static_layer",          # 静态地图层
        "ultrasonic_layer",      # 超声波障碍物层
        "obstacle_layer",        # 激光雷达等其他障碍物
        "inflation_layer"        # 障碍物膨胀层
      ]

      # 各层配置...
      ultrasonic_layer:
        plugin: "ultrasonic_costmap_layer/UltrasonicLayer"
        # ... (参见前面的配置参数)
```

### 层融合策略

本插件在 `updateCosts()` (cpp:484-551) 中实现了智能层融合：

1. **清除策略**（FREE_SPACE）：
   - 仅在主 costmap 为 `NO_INFORMATION` 或已有 `FREE_SPACE` 时清除
   - 不会清除其他层标记的障碍物或膨胀区域

2. **标记策略**（LETHAL_OBSTACLE）：
   - 仅在新代价高于现有代价时更新
   - 使用 "只增不减" 的安全策略

这确保了与其他 costmap 层的安全协作。

## 技术细节

### 线程安全

使用 `std::mutex` 保护共享数据（cpp:81-82）：
```cpp
std::mutex ultrasonic_mutex_;
std::list<car_chassis::msg::Ultrasonic> ultrasonic_msgs_buffer_;
```

消息缓冲在订阅回调和更新函数之间同步。

### 性能优化

1. **批量处理**：消息缓冲后批量处理（cpp:238-242）
2. **边界框裁剪**：只更新受影响的区域（cpp:377-384）
3. **早期退出**：无效读数立即返回（cpp:262-264）

### 内存管理

- 内部 costmap 继承自 `Costmap2D`，自动管理内存
- 消息缓冲使用 `std::list`，高效插入和清除

## 许可证

BSD-3-Clause

## 维护者

mini <mini@todo.todo>

## 相关包

- **ultrasonic_viz**: 超声波传感器可视化包
- **car_chassis**: 包含 Ultrasonic.msg 定义
- **nav2_costmap_2d**: Nav2 代价地图框架

## 参考资料

- [Nav2 Costmap Documentation](https://navigation.ros.org/configuration/packages/costmap-plugins/index.html)
- [Writing a New Costmap2D Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)
- 传感器模型基于经典的超声波传感器概率模型

## 更新日志

### v1.0.0
- 初始版本
- 支持三传感器（左、中、右）
- 概率传感器模型
- 射线清除功能
- 传感器位置偏移支持
