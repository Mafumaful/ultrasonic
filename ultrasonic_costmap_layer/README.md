# Ultrasonic Costmap Layer

Nav2 costmap plugin that creates arc-shaped obstacles from ultrasonic sensor data for the Cyber Dog Mini robot.

## Overview

This package provides a costmap layer plugin for ROS2 Navigation2 (Nav2) that subscribes to ultrasonic sensor data and generates arc-shaped obstacles in the local costmap. The plugin is designed to work with the `car_chassis/msg/Ultrasonic` message type, which provides readings from three ultrasonic sensors (left, middle, right).

## Features

- **Arc-Shaped Obstacles**: Creates realistic arc-shaped costmap obstacles based on ultrasonic sensor readings
- **Three Sensors**: Supports left, middle, and right ultrasonic sensors with configurable angles
- **Configurable Parameters**: All sensor parameters (angles, ranges, FOV) can be configured via YAML
- **Integration with ultrasonic_viz**: Uses the same configuration format as the ultrasonic visualization package
- **Nav2 Compatible**: Full integration with Nav2 navigation stack

## Installation

### Dependencies

- ROS2 Humble
- Nav2 (nav2_costmap_2d)
- car_chassis package (for Ultrasonic.msg)
- tf2

### Build

```bash
cd ~/Cyber_dog_mini
colcon build --packages-select ultrasonic_costmap_layer
source install/setup.bash
```

## Usage

### As Part of Nav2 Configuration

Add the ultrasonic layer to your Nav2 costmap configuration:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["ultrasonic_layer", "inflation_layer"]

      ultrasonic_layer:
        plugin: "ultrasonic_costmap_layer/UltrasonicLayer"
        enabled: true
        ultrasonic_topic: "/ultrasonic"
        sensor_angle_left: 45.0
        sensor_angle_mid: 0.0
        sensor_angle_right: -45.0
        min_range: 50.0
        max_range: 4000.0
        distance_scale: 1.0
        sensor_fov: 30.0
        arc_thickness: 0.15
```

### Testing the Plugin

To test the ultrasonic costmap layer independently:

```bash
# Terminal 1: Start the test costmap node
ros2 launch ultrasonic_costmap_layer test_ultrasonic_costmap.launch.py

# Terminal 2: Publish some test ultrasonic data
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic "{header: {frame_id: 'base_link'}, left: 1000, mid: 1500, right: 800}" --once

# Terminal 3: Visualize in RViz
rviz2
```

In RViz:
1. Set Fixed Frame to `odom`
2. Add a `Map` display
3. Set topic to `/ultrasonic_costmap`
4. You should see arc-shaped obstacles at the sensor readings

### Verify Costmap Generation

To check if the costmap is being published:

```bash
# Check if costmap topic exists
ros2 topic list | grep costmap

# Echo costmap updates
ros2 topic echo /ultrasonic_costmap_updates

# Check if the ultrasonic layer is loaded
ros2 param list | grep ultrasonic
```

## Configuration Parameters

All parameters are compatible with the ultrasonic_viz configuration format.

### Topic Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ultrasonic_topic` | string | `/ultrasonic` | Topic to subscribe for ultrasonic data |
| `enabled` | bool | `true` | Enable/disable the layer |

### Sensor Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_angle_left` | double | 45.0 | Left sensor angle (degrees, counter-clockwise) |
| `sensor_angle_mid` | double | 0.0 | Middle sensor angle (degrees) |
| `sensor_angle_right` | double | -45.0 | Right sensor angle (degrees) |

### Range Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | double | 50.0 | Minimum valid range (mm) |
| `max_range` | double | 4000.0 | Maximum valid range (mm) |
| `distance_scale` | double | 1.0 | Scale factor: real_distance_mm = sensor_value × distance_scale |

### Arc Visualization Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_fov` | double | 30.0 | Field of view for each sensor (degrees) |
| `arc_thickness` | double | 0.15 | Thickness of the arc obstacle (meters) |
| `no_readings_timeout` | double | 2.0 | Timeout for sensor data (seconds) |

## How It Works

### Arc Generation Algorithm

For each valid ultrasonic sensor reading:

1. **Calculate obstacle position**: Using sensor angle and distance
2. **Define arc parameters**: Based on sensor FOV and arc thickness
3. **Sample arc points**: Generate points along the arc
4. **Mark costmap cells**: Set cells to LETHAL_OBSTACLE cost

The arc is created by:
- Sampling points along the sensor's field of view
- Creating multiple radial samples to achieve arc thickness
- Marking all corresponding costmap cells as obstacles

### Coordinate System

- Sensor angles are relative to the robot's forward direction (base_link)
- Positive angles are counter-clockwise
- Distance is calculated from the robot's center

## Integration with Navigation

This plugin integrates seamlessly with Nav2:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05

      plugins: [
        "static_layer",
        "ultrasonic_layer",      # Add ultrasonic obstacles
        "obstacle_layer",         # Other obstacle sources
        "inflation_layer"         # Inflate obstacles
      ]
```

## Tuning Guide

### Adjusting Arc Thickness

If obstacles are too thin or thick:

```yaml
arc_thickness: 0.2  # Increase for thicker arcs
```

### Adjusting Sensor FOV

If the arc is too wide or narrow:

```yaml
sensor_fov: 25.0  # Decrease for narrower arc
```

### Calibrating Distance Scale

To match real-world distances:

1. Measure actual distance to an object (in mm)
2. Record sensor reading value
3. Calculate: `distance_scale = actual_distance / sensor_value`

```yaml
distance_scale: 2.5  # Example: sensor_value=1000 → 2500mm
```

## Troubleshooting

### No obstacles appear in costmap

1. Check if ultrasonic data is being published:
   ```bash
   ros2 topic echo /ultrasonic
   ```

2. Verify the plugin is loaded:
   ```bash
   ros2 param get /local_costmap plugins
   ```

3. Check distance readings are within valid range:
   ```bash
   # Readings should be between min_range and max_range
   ros2 topic echo /ultrasonic --field left
   ```

### Obstacles at wrong positions

- Verify sensor angles match your hardware configuration
- Check `distance_scale` parameter
- Ensure TF frames (base_link, odom) are correctly published

### Plugin not loading

```bash
# Check if plugin is registered
ros2 pkg prefix ultrasonic_costmap_layer
ls install/ultrasonic_costmap_layer/share/ultrasonic_costmap_layer/

# Rebuild with verbose output
colcon build --packages-select ultrasonic_costmap_layer --event-handlers console_direct+
```

## Example: Testing with Fake Data

Publish test data to see arcs in RViz:

```bash
# Obstacle at 1.5m in front
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 2000, mid: 1500, right: 2000}" --rate 10

# Obstacle at 1m on left side
ros2 topic pub /ultrasonic car_chassis/msg/Ultrasonic \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   left: 1000, mid: 3000, right: 3000}" --rate 10
```

## Files Structure

```
ultrasonic_costmap_layer/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata
├── README.md                         # This file
├── ultrasonic_layer.xml             # Plugin description
├── include/
│   └── ultrasonic_costmap_layer/
│       └── ultrasonic_layer.hpp     # Header file
├── src/
│   └── ultrasonic_layer.cpp         # Implementation
├── config/
│   └── ultrasonic_layer_params.yaml # Example configuration
└── test/
    └── test_ultrasonic_costmap.launch.py  # Test launch file
```

## License

BSD-3-Clause

## Maintainer

mini <mini@todo.todo>

## Related Packages

- `ultrasonic_viz`: Visualization package for ultrasonic sensors
- `car_chassis`: Package containing Ultrasonic.msg definition
- `nav2_costmap_2d`: Nav2 costmap framework
