# 超声波 Costmap 层调参指南

本文档详细说明了超声波 costmap 层的所有参数及其调优方法。

## 目录
- [快速开始](#快速开始)
- [基础参数](#基础参数)
- [传感器配置](#传感器配置)
- [概率模型参数](#概率模型参数)
- [射线清除参数](#射线清除参数)
- [拖影消除（环带清除）](#拖影消除环带清除)
- [范围限制（幽灵障碍物）](#范围限制幽灵障碍物)
- [常见问题与解决方案](#常见问题与解决方案)
- [调参工作流](#调参工作流)

---

## 快速开始

### 最小配置（适用于大多数场景）

```yaml
ultrasonic_layer:
  plugin: "ultrasonic_costmap_layer::UltrasonicLayer"
  enabled: true

  # 基础配置
  ultrasonic_topic: /ultrasonic
  min_range: 50.0      # mm，传感器最小有效距离
  max_range: 4000.0    # mm，传感器最大有效距离
  sensor_fov: 30.0     # degrees，传感器视场角

  # 阈值（影响障碍物判定）
  mark_threshold: 0.8   # 高于此值标记为障碍物
  clear_threshold: 0.2  # 低于此值标记为自由空间
```

---

## 基础参数

### `enabled`
- **类型**: bool
- **默认值**: `true`
- **说明**: 是否启用超声波层
- **影响**: 关闭后，超声波数据不会影响 costmap

---

### `ultrasonic_topic`
- **类型**: string
- **默认值**: `/ultrasonic`
- **说明**: 超声波话题名称
- **要求**: 消息类型必须为 `car_chassis::msg::Ultrasonic`，包含 `left`, `mid`, `right` 三个整数字段（单位：毫米）

---

### `no_readings_timeout`
- **类型**: double (秒)
- **默认值**: `2.0`
- **说明**: 超过此时间未收到数据则警告
- **影响**:
  - 设置太小：频繁误报
  - 设置太大：传感器故障时无法及时发现
- **推荐值**: `2.0 - 5.0`

---

## 传感器配置

### 量程参数

#### `min_range`
- **类型**: double (毫米)
- **默认值**: `50.0`
- **说明**: 传感器最小有效测距
- **影响**: 小于此值的读数被忽略
- **调参提示**:
  - 根据传感器数据手册设置
  - 如果近距离障碍物漏检，可适当减小

#### `max_range`
- **类型**: double (毫米)
- **默认值**: `4000.0`
- **说明**: 传感器最大有效测距
- **影响**: 大于此值的读数被忽略
- **调参提示**:
  - 根据传感器数据手册设置
  - 超声波实际量程通常较短，不建议设置过大

#### `distance_scale`
- **类型**: double
- **默认值**: `1.0`
- **说明**: 距离缩放因子（真实距离 = 传感器值 × scale）
- **何时使用**: 传感器读数需要校准时

---

### 视场角参数

#### `sensor_fov`
- **类型**: double (度)
- **默认值**: `30.0`
- **说明**: 传感器的视场角（FOV）
- **影响**:
  - 增大：障碍物覆盖范围更宽，但可能误判侧边物体
  - 减小：障碍物覆盖范围更窄，但可能漏检
- **调参提示**:
  - 查阅传感器数据手册获取真实 FOV
  - HC-SR04 典型值：15°-30°
  - 如果经常漏检侧边障碍物，可适当增大

---

### 传感器安装参数

#### `sensor_angle_left` / `sensor_angle_mid` / `sensor_angle_right`
- **类型**: double (度)
- **默认值**: `45.0` / `0.0` / `-45.0`
- **说明**: 传感器相对于 base_link x 轴的安装角度
- **坐标系**: 逆时针为正
- **示例**:
  ```
       左(45°)  中(0°)  右(-45°)
           \      |      /
            \     |     /
             机器人前方
  ```

#### `sensor_left_tx` / `sensor_left_ty` (同理 mid/right)
- **类型**: double (米)
- **默认值**: `0.0` / `0.0`
- **说明**: 传感器在 base_link 坐标系中的位置偏移
- **重要性**: ⭐⭐⭐⭐⭐
- **影响**: 偏移不准确会导致障碍物位置错误
- **测量方法**:
  1. 测量传感器到 base_link 原点的距离
  2. x 轴正方向为前，y 轴正方向为左
  3. 精确到厘米级

---

## 概率模型参数

超声波层使用概率模型来表示障碍物的不确定性。理解这些参数需要一些背景知识：

### 概率模型原理

每个单元格维护一个占据概率 `[0, 1]`：
- `0.0`：确定为自由空间
- `0.5`：未知（默认值）
- `1.0`：确定为障碍物

### `mark_threshold`
- **类型**: double
- **默认值**: `0.8`
- **说明**: 概率高于此值时，标记为 LETHAL_OBSTACLE (254)
- **影响**:
  - **增大（如 0.9）**：
    - ✅ 减少误报（更确定才标记）
    - ❌ 可能漏检真实障碍物
  - **减小（如 0.6）**：
    - ✅ 更灵敏，不易漏检
    - ❌ 可能将不确定区域标记为障碍
- **推荐值**: `0.7 - 0.85`
- **调参场景**:
  - 误报太多 → 提高到 0.85-0.9
  - 经常漏检 → 降低到 0.6-0.7

---

### `clear_threshold`
- **类型**: double
- **默认值**: `0.2`
- **说明**: 概率低于此值时，标记为 FREE_SPACE (0)
- **影响**:
  - **增大（如 0.4）**：
    - ✅ 更积极清除障碍物
    - ❌ 可能误清真实障碍物
  - **减小（如 0.1）**：
    - ✅ 更保守，不轻易清除
    - ❌ 旧障碍物可能残留
- **推荐值**: `0.15 - 0.35`
- **重要原则**: `clear_threshold + mark_threshold` 应该远小于 1.0，留出"未知"区间

---

### `phi_v`
- **类型**: double
- **默认值**: `1.2`
- **说明**: 距离衰减函数的拐点，控制障碍物区域的锐度
- **影响**:
  - **增大（如 1.5）**：
    - 障碍物区域更"软"，过渡更平滑
    - 障碍物后方的高概率区域更大
  - **减小（如 0.8）**：
    - 障碍物区域更"硬"，边界更锐利
    - 障碍物更集中在测距点附近
- **推荐值**: `1.0 - 1.5`
- **高级调参**: 通常不需要修改

---

### `inflate_cone`
- **类型**: double
- **默认值**: `1.0`
- **说明**: 扇形区域膨胀系数
- **影响**:
  - `> 1.0`：扩大扇形区域（障碍物覆盖更宽）
  - `< 1.0`：缩小扇形区域（障碍物覆盖更窄）
- **推荐值**: `0.8 - 1.2`
- **使用场景**:
  - 障碍物检测范围不够 → 增大到 1.1-1.2
  - 误将侧边物体判为障碍 → 减小到 0.8-0.9

---

## 射线清除参数

射线清除功能用于清除传感器到障碍物之间的自由空间。

### `enable_ray_clear`
- **类型**: bool
- **默认值**: `true`
- **说明**: 是否启用射线清除
- **影响**:
  - `true`：清除传感器到障碍物之间的区域
  - `false`：不清除，可能导致"拖影"
- **推荐**: 保持开启

---

### `ray_clear_margin`
- **类型**: double (米)
- **默认值**: `0.10`
- **说明**: 清除到障碍物前的安全边距
- **影响**:
  - **增大（如 0.2）**：
    - ✅ 障碍物前方留更多缓冲
    - ❌ 可能保留过多"拖影"
  - **减小（如 0.05）**：
    - ✅ 更彻底清除
    - ❌ 可能误清障碍物边缘
- **推荐值**: `0.05 - 0.15`

---

### `clear_on_max_reading`
- **类型**: bool
- **默认值**: `false`
- **说明**: 当读数达到最大量程时，是否清除整个射线
- **影响**:
  - `true`：假设前方无障碍物，清除整个射线（可能误清）
  - `false`：不清除（更保守）
- **推荐**: 保持 `false`

---

## 拖影消除（环带清除）

当障碍物从远处移近或被移走时，旧的障碍物标记可能残留，形成"拖影"。环带清除功能自动解决这个问题。

### 工作原理

```
时刻 t1: 检测到障碍物在 3m → 记录 last_range = 3m
时刻 t2: 检测到障碍物在 1.5m → 清除 [1.5m, 3m] 环带 → 更新 last_range = 1.5m
时刻 t3: 检测到障碍物在 1m → 清除 [1m, 1.5m] 环带 → 更新 last_range = 1m
```

### 参数（自动启用，无需配置）

环带清除功能**自动启用**，使用现有的 `ray_clear_margin` 参数：
- 内半径 = `current_range + ray_clear_margin`
- 外半径 = `last_range`
- 范围：传感器 FOV 内

### 调参提示

如果环带清除效果不理想：
1. **拖影清除不彻底** → 检查 `ray_clear_margin` 是否过大
2. **误清真实障碍物** → 增大 `ray_clear_margin`（留更多安全边距）

---

## 范围限制（幽灵障碍物）

在 rolling_window 模式的 local_costmap 中，机器人移动后，远处的旧障碍物可能残留，形成"幽灵障碍物"。范围限制功能自动清除超出有效范围的障碍物。

### `enable_out_of_range_clearing`
- **类型**: bool
- **默认值**: `true`
- **说明**: 是否启用范围外清除
- **影响**:
  - `true`：自动清除超出范围的障碍物（推荐）
  - `false`：不清除，可能导致"幽灵障碍物"
- **推荐**: 保持开启

---

### `clearing_range`
- **类型**: double (米)
- **默认值**: `-1.0`
- **说明**: 有效范围半径
  - `-1.0`：自动使用 `min(costmap_width, costmap_height) / 2`
  - 其他正值：使用指定半径
- **影响**:
  - **减小（如 3.0）**：
    - ✅ 更激进清除，保留范围更小
    - ❌ 可能误清仍在视野内的障碍物
  - **增大（如 7.0）**：
    - ✅ 更保守，保留范围更大
    - ❌ "幽灵障碍物"可能残留更久
- **推荐配置**:
  - **10m × 10m local_costmap**: `-1.0`（自动 5m）或 `4.0 - 6.0`
  - **5m × 5m local_costmap**: `-1.0`（自动 2.5m）或 `2.0 - 3.0`

---

## 常见问题与解决方案

### 问题 1：障碍物检测不到（漏检）

**可能原因与解决方案**:

1. **阈值过高**
   - 检查 `mark_threshold`，尝试降低到 0.6-0.7

2. **视场角不够**
   - 增大 `sensor_fov`（如 30° → 35°）
   - 检查 `sensor_angle_*` 是否正确

3. **传感器位置偏移错误**
   - 重新测量 `sensor_*_tx/ty`，确保精确

4. **距离超出量程**
   - 检查实际测距是否在 `[min_range, max_range]` 内

---

### 问题 2：误报太多（假阳性）

**可能原因与解决方案**:

1. **阈值过低**
   - 提高 `mark_threshold` 到 0.85-0.9

2. **扇形区域过大**
   - 减小 `inflate_cone` 到 0.8-0.9
   - 减小 `sensor_fov`

3. **地面/天花板反射**
   - 检查传感器安装角度
   - 调整传感器物理安装

---

### 问题 3：拖影严重（障碍物移走后仍显示）

**可能原因与解决方案**:

1. **清除阈值过低**
   - 提高 `clear_threshold` 到 0.3-0.4

2. **射线清除未启用**
   - 确保 `enable_ray_clear: true`

3. **环带清除边距过大**
   - 减小 `ray_clear_margin` 到 0.05

---

### 问题 4：远处有"幽灵障碍物"

**可能原因与解决方案**:

1. **范围限制未启用**
   - 确保 `enable_out_of_range_clearing: true`

2. **清除范围过大**
   - 减小 `clearing_range`（如 `-1.0` → `3.0`）

---

### 问题 5：障碍物抖动（频繁出现/消失）

**可能原因与解决方案**:

1. **阈值区间太窄**
   - 拉大 `mark_threshold` 和 `clear_threshold` 的差距
   - 例如：`clear: 0.2, mark: 0.85`

2. **传感器噪声大**
   - 检查传感器硬件
   - 增大 `mark_threshold`

---

### 问题 6：障碍物位置偏移

**可能原因与解决方案**:

1. **传感器位置偏移错误** ⭐最常见原因⭐
   - 精确测量并设置 `sensor_*_tx/ty`

2. **传感器角度错误**
   - 检查 `sensor_angle_*` 是否正确

3. **TF 变换问题**
   - 检查 `base_link` 到 `map` 的 TF 是否正确

---

## 调参工作流

### 步骤 1：验证基础配置

```yaml
# 确保传感器基本参数正确
min_range: 50.0       # 查阅数据手册
max_range: 4000.0     # 查阅数据手册
sensor_fov: 30.0      # 查阅数据手册

# 确保传感器位置精确（⭐最重要⭐）
sensor_left_tx: ???   # 精确测量
sensor_left_ty: ???
sensor_angle_left: ??? # 精确测量
```

**验证方法**:
1. 放置一个障碍物在传感器前方已知位置
2. 查看 RViz 中障碍物是否出现在正确位置
3. 如果偏移，调整 `sensor_*_tx/ty`

---

### 步骤 2：调整检测灵敏度

从保守设置开始：

```yaml
mark_threshold: 0.85   # 高阈值，减少误报
clear_threshold: 0.2   # 低阈值，谨慎清除
```

**调整策略**:
- **漏检太多** → 降低 `mark_threshold`（0.85 → 0.75 → 0.65）
- **误报太多** → 提高 `mark_threshold`（0.85 → 0.9）

---

### 步骤 3：调整清除行为

```yaml
enable_ray_clear: true     # 启用射线清除
ray_clear_margin: 0.10     # 保留 10cm 安全边距
clear_threshold: 0.2       # 清除阈值
```

**调整策略**:
- **拖影严重** → 提高 `clear_threshold`（0.2 → 0.3）或减小 `ray_clear_margin`（0.1 → 0.05）
- **误清障碍物** → 降低 `clear_threshold`（0.2 → 0.15）或增大 `ray_clear_margin`（0.1 → 0.15）

---

### 步骤 4：启用高级功能

```yaml
# 拖影消除（自动启用）
# 使用现有的 ray_clear_margin 参数

# 幽灵障碍物清除
enable_out_of_range_clearing: true
clearing_range: -1.0  # 自动模式
```

---

### 步骤 5：微调

根据实际表现微调：

```yaml
# 扇形区域大小
inflate_cone: 1.0      # 障碍物覆盖范围不够 → 1.1-1.2
                       # 误检侧边物体 → 0.8-0.9

# 概率模型锐度（通常不需要改）
phi_v: 1.2            # 障碍物区域过硬 → 1.3-1.5
                       # 障碍物区域过软 → 0.9-1.1
```

---

## 推荐配置模板

### 保守配置（适合人员密集环境）

```yaml
ultrasonic_layer:
  plugin: "ultrasonic_costmap_layer::UltrasonicLayer"
  enabled: true

  # 基础配置
  ultrasonic_topic: /ultrasonic
  min_range: 50.0
  max_range: 4000.0
  sensor_fov: 30.0

  # 保守阈值（减少误报）
  mark_threshold: 0.85
  clear_threshold: 0.15

  # 射线清除（更多安全边距）
  enable_ray_clear: true
  ray_clear_margin: 0.15

  # 范围限制
  enable_out_of_range_clearing: true
  clearing_range: -1.0

  # 传感器位置（需精确测量）
  sensor_angle_left: 45.0
  sensor_angle_mid: 0.0
  sensor_angle_right: -45.0
  sensor_left_tx: 0.355
  sensor_left_ty: 0.155
  sensor_mid_tx: 0.115
  sensor_mid_ty: 0.0
  sensor_right_tx: 0.355
  sensor_right_ty: -0.155
```

---

### 激进配置（适合开阔环境）

```yaml
ultrasonic_layer:
  plugin: "ultrasonic_costmap_layer::UltrasonicLayer"
  enabled: true

  # 基础配置
  ultrasonic_topic: /ultrasonic
  min_range: 50.0
  max_range: 4000.0
  sensor_fov: 35.0  # 更大视场角

  # 激进阈值（更灵敏）
  mark_threshold: 0.65
  clear_threshold: 0.35

  # 射线清除（更少安全边距）
  enable_ray_clear: true
  ray_clear_margin: 0.05

  # 范围限制（更小范围）
  enable_out_of_range_clearing: true
  clearing_range: 3.0

  # 扇形膨胀
  inflate_cone: 1.1

  # 传感器位置（需精确测量）
  sensor_angle_left: 45.0
  sensor_angle_mid: 0.0
  sensor_angle_right: -45.0
  sensor_left_tx: 0.355
  sensor_left_ty: 0.155
  sensor_mid_tx: 0.115
  sensor_mid_ty: 0.0
  sensor_right_tx: 0.355
  sensor_right_ty: -0.155
```

---

## 调试技巧

### 1. 使用 RViz 可视化

在 RViz 中添加：
- `Costmap` 显示（`/local_costmap/costmap`）
- 查看超声波层的障碍物分布

### 2. 查看日志

```bash
ros2 topic echo /rosout | grep UltrasonicLayer
```

关键日志：
- 初始化信息（参数加载）
- 传感器读数（前 5 条）
- 超时警告

### 3. 监控传感器数据

```bash
ros2 topic echo /ultrasonic
```

检查：
- 数据是否正常更新
- 距离值是否在预期范围内

### 4. 检查 TF 变换

```bash
ros2 run tf2_ros tf2_echo map base_link
```

确保 base_link 到 map 的变换正常。

---

## 参数速查表

| 参数 | 默认值 | 范围 | 影响 |
|------|--------|------|------|
| `mark_threshold` | 0.8 | 0.5-0.95 | 障碍物判定灵敏度 |
| `clear_threshold` | 0.2 | 0.05-0.4 | 清除灵敏度 |
| `sensor_fov` | 30.0° | 15-45° | 检测范围宽度 |
| `ray_clear_margin` | 0.10m | 0.05-0.2 | 清除安全边距 |
| `clearing_range` | -1.0 | -1.0 或 2.0-10.0 | 幽灵障碍物清除范围 |
| `inflate_cone` | 1.0 | 0.8-1.2 | 扇形区域大小 |
| `phi_v` | 1.2 | 0.8-1.5 | 障碍物区域锐度 |

---

## 总结

**核心调参原则**:
1. ⭐ **传感器位置必须精确** - 这是最重要的！
2. 从保守配置开始，逐步放宽
3. `mark_threshold` 和 `clear_threshold` 差距要大
4. 启用射线清除和范围限制
5. 在 RViz 中实时观察效果

**常见错误**:
- ❌ 传感器位置偏移未精确测量
- ❌ `mark_threshold` 和 `clear_threshold` 过于接近
- ❌ 关闭射线清除导致拖影
- ❌ 未启用范围限制导致幽灵障碍物

**调参顺序**:
1. 基础配置（量程、FOV、传感器位置）
2. 检测灵敏度（mark/clear threshold）
3. 清除行为（ray clear）
4. 高级功能（范围限制）
5. 微调（inflate_cone, phi_v）

---

## 技术支持

如有问题，请检查：
1. 日志输出（`ros2 topic echo /rosout`）
2. 传感器数据（`ros2 topic echo /ultrasonic`）
3. TF 变换（`ros2 run tf2_ros tf2_echo map base_link`）
4. RViz 可视化

常见问题请参考本文档的"常见问题与解决方案"章节。
