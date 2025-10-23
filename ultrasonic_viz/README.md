# Ultrasonic Visualization (ultrasonic_viz)

ROS2 package for visualizing ultrasonic sensor data from the Cyber Dog Mini robot.

## Overview

This package provides a ROS2 node that subscribes to ultrasonic sensor data and publishes visualization markers for RViz. The visualization includes:
- **Sensor cones**: Show the field of view for each ultrasonic sensor
- **Obstacle markers**: Display detected obstacles at their actual positions

## Features

- Real-time visualization of 3 ultrasonic sensors (left, middle, right)
- Configurable sensor parameters via YAML file
- Adjustable sensor angles, ranges, and distance scaling
- Color-coded visualization (green cones for sensor FOV, red spheres for obstacles)
- Easy parameter tuning without code modification

## Installation

1. Make sure you have the `car_chassis` package built (contains `Ultrasonic.msg`)

2. Build this package:
   ```bash
   cd ~/Cyber_dog_mini
   colcon build --packages-select ultrasonic_viz
   source install/setup.bash
   ```

## Usage

### Launch with default parameters

```bash
ros2 launch ultrasonic_viz ultrasonic_viz.launch.py
```

### Launch with custom config file

```bash
ros2 launch ultrasonic_viz ultrasonic_viz.launch.py config_file:=/path/to/your/config.yaml
```

### Run node directly

```bash
ros2 run ultrasonic_viz ultrasonic_viz_node --ros-args --params-file /path/to/config.yaml
```

## Configuration

All parameters can be configured in `config/ultrasonic_params.yaml`:

### Topic Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ultrasonic_topic` | string | `/ultrasonic` | Topic to subscribe for ultrasonic data |
| `marker_topic` | string | `/ultrasonic_markers` | Topic to publish visualization markers |
| `frame_id` | string | `base_link` | TF frame for visualization |

### Sensor Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_angles.left` | double | 45.0 | Left sensor angle in degrees |
| `sensor_angles.mid` | double | 0.0 | Middle sensor angle in degrees |
| `sensor_angles.right` | double | -45.0 | Right sensor angle in degrees |

**Note**: Angles are relative to the robot's forward direction, with counter-clockwise being positive.

### Range Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | double | 50.0 | Minimum detection range (mm) |
| `max_range` | double | 4000.0 | Maximum detection range (mm) |
| `distance_scale` | double | 1.0 | Scale factor to convert sensor value to real distance (mm) |

**Distance Calculation**: `real_distance_mm = sensor_value × distance_scale`

### Visualization Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_lifetime` | double | 0.2 | Marker display duration (seconds) |
| `cone_color.{r,g,b,a}` | double | 0.0, 1.0, 0.0, 0.3 | Sensor cone color (RGBA) |
| `obstacle_color.{r,g,b,a}` | double | 1.0, 0.0, 0.0, 0.8 | Obstacle marker color (RGBA) |
| `publish_rate` | double | 10.0 | Visualization update rate (Hz) |

## RViz Visualization

1. Open RViz:
   ```bash
   rviz2
   ```

2. Set the **Fixed Frame** to `base_link` (or your configured `frame_id`)

3. Add a **MarkerArray** display:
   - Click **Add** button
   - Select **By topic** tab
   - Choose `/ultrasonic_markers` → **MarkerArray**

4. You should now see:
   - Green transparent cones showing sensor fields of view
   - Red spheres at detected obstacle positions

## Tuning Parameters

### Adjusting Sensor Angles

If your sensors are mounted at different angles, modify in `config/ultrasonic_params.yaml`:

```yaml
sensor_angles:
  left: 60.0      # Change to your actual angle
  mid: 0.0        # Forward facing
  right: -60.0    # Change to your actual angle
```

### Calibrating Distance Scale

The `distance_scale` parameter converts raw sensor values to real-world distances:

1. Measure actual distance to an obstacle (in mm)
2. Record the sensor reading value
3. Calculate: `distance_scale = actual_distance_mm / sensor_value`
4. Update in config file:

```yaml
distance_scale: 2.5  # Example: if sensor_value=1000 gives 2500mm
```

### Adjusting Range Limits

Set appropriate min/max ranges based on your sensor specifications:

```yaml
min_range: 100.0     # Objects closer than this are ignored
max_range: 3000.0    # Maximum reliable detection range
```

## Topics

### Subscribed Topics

- `~/ultrasonic` (`car_chassis/msg/Ultrasonic`): Ultrasonic sensor data
  ```
  std_msgs/Header header
  uint16 left    # Left sensor reading
  uint16 mid     # Middle sensor reading
  uint16 right   # Right sensor reading
  ```

### Published Topics

- `~/ultrasonic_markers` (`visualization_msgs/msg/MarkerArray`): Visualization markers for RViz

## Example Configuration

Here's an example configuration for sensors mounted at 30-degree intervals:

```yaml
ultrasonic_viz_node:
  ros__parameters:
    sensor_angles:
      left: 30.0
      mid: 0.0
      right: -30.0

    min_range: 100.0
    max_range: 2500.0
    distance_scale: 2.0

    cone_color:
      r: 0.0
      g: 0.8
      b: 1.0
      a: 0.4
```

## Troubleshooting

### No markers appear in RViz

1. Check if ultrasonic data is being published:
   ```bash
   ros2 topic echo /ultrasonic
   ```

2. Verify node is running:
   ```bash
   ros2 node list | grep ultrasonic_viz
   ```

3. Check marker topic:
   ```bash
   ros2 topic echo /ultrasonic_markers
   ```

### Obstacles appear at wrong distances

- Adjust the `distance_scale` parameter based on actual measurements
- Verify `min_range` and `max_range` settings

### Sensors pointing in wrong directions

- Check and adjust `sensor_angles` in the config file
- Remember: angles are in degrees, counter-clockwise positive

## License

MIT

## Maintainer

- mini <mini@todo.todo>

## Dependencies

- ROS2 Humble
- rclpy
- visualization_msgs
- geometry_msgs
- car_chassis (custom messages)
