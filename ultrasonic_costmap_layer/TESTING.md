# Testing Ultrasonic Costmap Layer

本文档说明如何测试超声波 costmap 插件。

## 快速测试

### 方法 1: 使用测试 Launch 文件

这是最简单的测试方法，可以验证 costmap 是否正确生成。

```bash
# Terminal 1: 启动 costmap 节点
source install/setup.bash
ros2 launch ultrasonic_costmap_layer test_ultrasonic_costmap.launch.py

# Terminal 2: 发布测试超声波数据
source install/setup.bash
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 1000, mid: 1500, right: 800}" --rate 10

# Terminal 3: 可视化 (RViz)
rviz2
```

在 RViz 中:
1. 设置 Fixed Frame 为 `base_link`
2. 添加 `Map` 显示
3. 设置 Topic 为 `/ultrasonic_costmap`
4. 你应该能看到 3 个弧形障碍物

### 方法 2: 验证 Costmap 数据

检查 costmap 是否正在发布:

```bash
source install/setup.bash

# 检查 topic 列表
ros2 topic list | grep costmap

# 应该看到:
# /ultrasonic_costmap
# /ultrasonic_costmap_updates

# 查看 costmap 数据
ros2 topic echo /ultrasonic_costmap --once

# 查看更新信息
ros2 topic echo /ultrasonic_costmap_updates
```

## 详细测试步骤

### 步骤 1: 验证插件安装

```bash
cd ~/Cyber_dog_mini
source install/setup.bash

# 运行验证脚本
bash src/traj_devel/ultrasonic/ultrasonic_costmap_layer/test/verify_plugin.sh
```

预期输出应该显示所有检查项都通过 (✓)。

### 步骤 2: 测试不同距离的障碍物

```bash
# 测试 1: 近距离障碍物 (0.5m)
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 500, mid: 500, right: 500}" --rate 10

# 测试 2: 中距离障碍物 (1.5m)
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 1500, mid: 1500, right: 1500}" --rate 10

# 测试 3: 远距离障碍物 (3.0m)
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 3000, mid: 3000, right: 3000}" --rate 10

# 测试 4: 左侧有障碍物
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 800, mid: 3500, right: 3500}" --rate 10

# 测试 5: 右侧有障碍物
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {frame_id: 'base_link'}, left: 3500, mid: 3500, right: 800}" --rate 10
```

### 步骤 3: 检查参数配置

```bash
# 查看所有参数
ros2 param list | grep ultrasonic

# 查看特定参数
ros2 param get /local_costmap ultrasonic_layer.sensor_angle_left
ros2 param get /local_costmap ultrasonic_layer.min_range
ros2 param get /local_costmap ultrasonic_layer.distance_scale
```

### 步骤 4: 动态修改参数 (可选)

```bash
# 修改传感器角度
ros2 param set /local_costmap ultrasonic_layer.sensor_angle_left 60.0

# 修改距离比例
ros2 param set /local_costmap ultrasonic_layer.distance_scale 2.0

# 注意: 修改参数后需要重启节点才能生效
```

## 验证 Costmap 生成

### 检查 1: Topic 数据流

```bash
# 检查超声波数据是否发布
ros2 topic hz /ultrasonic

# 应该显示发布频率 (如果使用 --rate 10，应该约为 10 Hz)

# 检查 costmap 更新频率
ros2 topic hz /ultrasonic_costmap_updates

# 应该显示更新频率 (根据配置中的 update_frequency)
```

### 检查 2: 日志输出

查看节点日志确认插件已加载:

```bash
# 在启动 costmap 的终端中，你应该看到类似的日志:
# [INFO] [local_costmap]: UltrasonicLayer: Initialized. Subscribing to /ultrasonic
# [INFO] [local_costmap]: Sensor angles (deg): left=45.0, mid=0.0, right=-45.0
# [INFO] [local_costmap]: Range: 0.050 - 4.000 m, scale: 1.00, FOV: 30.0 deg
```

### 检查 3: RViz 可视化

在 RViz 中验证弧形障碍物:

1. **添加 Map 显示**:
   - Topic: `/ultrasonic_costmap`
   - Color Scheme: costmap

2. **添加 TF 显示**:
   - 验证 `base_link` 和 `odom` 的坐标系

3. **验证弧形**:
   - 弧形应该在传感器方向上
   - 弧形的半径应该对应传感器距离
   - 应该有 3 个独立的弧 (左、中、右)

## 常见问题排查

### 问题 1: 没有看到障碍物

```bash
# 检查超声波数据
ros2 topic echo /ultrasonic

# 检查距离是否在有效范围内 (50-4000 mm)
# 如果传感器值超出范围，不会生成障碍物
```

### 问题 2: 障碍物位置不对

```bash
# 检查 distance_scale 参数
ros2 param get /local_costmap ultrasonic_layer.distance_scale

# 检查传感器角度
ros2 param get /local_costmap ultrasonic_layer.sensor_angle_left
ros2 param get /local_costmap ultrasonic_layer.sensor_angle_mid
ros2 param get /local_costmap ultrasonic_layer.sensor_angle_right
```

### 问题 3: Costmap 节点无法启动

```bash
# 检查插件是否正确安装
bash src/traj_devel/ultrasonic/ultrasonic_costmap_layer/test/verify_plugin.sh

# 重新编译
colcon build --packages-select ultrasonic_costmap_layer
source install/setup.bash
```

### 问题 4: TF 错误

```bash
# 发布静态 TF (如果没有其他节点发布)
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 odom base_link
```

## 测试成功标准

插件测试成功的标准:

- ✓ 插件验证脚本所有检查通过
- ✓ Costmap 节点成功启动，没有错误
- ✓ 在 RViz 中能看到弧形障碍物
- ✓ 障碍物位置与传感器读数对应
- ✓ 改变传感器读数时，障碍物位置更新
- ✓ 无效读数 (超出范围) 不生成障碍物

## 下一步

测试成功后，可以将此插件集成到你的导航配置中:

1. 将 `ultrasonic_layer` 添加到你的 Nav2 配置文件
2. 调整参数以匹配你的硬件配置
3. 与其他 costmap 层 (如 obstacle_layer) 一起使用

参考 README.md 了解更多集成信息。
