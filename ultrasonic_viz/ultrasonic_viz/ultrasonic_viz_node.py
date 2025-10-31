#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from car_chassis.msg import Ultrasonic
import math


class UltrasonicVizNode(Node):
    """
    ROS2 node for visualizing ultrasonic sensor data.

    Subscribes to ultrasonic sensor data and publishes visualization markers
    showing sensor cones and detected obstacles.
    """

    def __init__(self):
        super().__init__('ultrasonic_viz_node')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ultrasonic_topic', '/ultrasonic'),
                ('marker_topic', '/ultrasonic_markers'),
                ('frame_id', 'base_link'),
                ('sensor_angles.left', 45.0),
                ('sensor_angles.mid', 0.0),
                ('sensor_angles.right', -45.0),
                ('sensor_positions.mid_tx', 0.10),
                ('sensor_positions.mid_ty', 0.00),
                ('sensor_positions.left_tx', 0.10),
                ('sensor_positions.left_ty', 0.15),
                ('sensor_positions.right_tx', 0.10),
                ('sensor_positions.right_ty', -0.15),
                ('min_range', 50.0),
                ('max_range', 4000.0),
                ('distance_scale', 1.0),
                ('marker_lifetime', 0.2),
                ('cone_color.r', 0.0),
                ('cone_color.g', 1.0),
                ('cone_color.b', 0.0),
                ('cone_color.a', 0.3),
                ('obstacle_color.r', 1.0),
                ('obstacle_color.g', 0.0),
                ('obstacle_color.b', 0.0),
                ('obstacle_color.a', 0.8),
                ('cone_height_scale', 1.0),
                ('cone_width_scale', 0.5),
                ('publish_rate', 10.0),
            ]
        )

        # Get parameters
        self.ultrasonic_topic = self.get_parameter('ultrasonic_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.sensor_angles = {
            'left': math.radians(self.get_parameter('sensor_angles.left').value),
            'mid': math.radians(self.get_parameter('sensor_angles.mid').value),
            'right': math.radians(self.get_parameter('sensor_angles.right').value),
        }

        self.sensor_positions = {
            'mid': {
                'x': self.get_parameter('sensor_positions.mid_tx').value,
                'y': self.get_parameter('sensor_positions.mid_ty').value,
            },
            'left': {
                'x': self.get_parameter('sensor_positions.left_tx').value,
                'y': self.get_parameter('sensor_positions.left_ty').value,
            },
            'right': {
                'x': self.get_parameter('sensor_positions.right_tx').value,
                'y': self.get_parameter('sensor_positions.right_ty').value,
            }
        }

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.distance_scale = self.get_parameter('distance_scale').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value

        self.cone_color = {
            'r': self.get_parameter('cone_color.r').value,
            'g': self.get_parameter('cone_color.g').value,
            'b': self.get_parameter('cone_color.b').value,
            'a': self.get_parameter('cone_color.a').value,
        }

        self.obstacle_color = {
            'r': self.get_parameter('obstacle_color.r').value,
            'g': self.get_parameter('obstacle_color.g').value,
            'b': self.get_parameter('obstacle_color.b').value,
            'a': self.get_parameter('obstacle_color.a').value,
        }

        self.cone_height_scale = self.get_parameter('cone_height_scale').value
        self.cone_width_scale = self.get_parameter('cone_width_scale').value

        # Configure QoS profile for sensor data
        # Using BEST_EFFORT reliability for better compatibility with sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Configure QoS profile for visualization markers
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber
        self.ultrasonic_sub = self.create_subscription(
            Ultrasonic,
            self.ultrasonic_topic,
            self.ultrasonic_callback,
            sensor_qos
        )

        # Create publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            marker_qos
        )

        # Store latest ultrasonic data
        self.latest_ultrasonic_data = None

        # Create timer for publishing markers
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_markers)

        # Message counter for debugging
        self.message_count = 0
        self.publish_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Ultrasonic Visualization Node started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribing to: {self.ultrasonic_topic}')
        self.get_logger().info(f'Publishing to: {self.marker_topic}')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Distance scale: {self.distance_scale}')
        self.get_logger().info(f'Range: {self.min_range} - {self.max_range} mm')
        self.get_logger().info(f'Sensor QoS: BEST_EFFORT')
        self.get_logger().info(f'Marker QoS: RELIABLE')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.get_logger().info('Sensor positions:')
        for sensor_name, pos in self.sensor_positions.items():
            self.get_logger().info(f'  {sensor_name}: x={pos["x"]:.3f}m, y={pos["y"]:.3f}m')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for ultrasonic messages...')

    def ultrasonic_callback(self, msg: Ultrasonic):
        """Callback for ultrasonic sensor data."""
        self.message_count += 1
        self.latest_ultrasonic_data = msg

        # Log first message
        if self.message_count == 1:
            self.get_logger().info('=' * 60)
            self.get_logger().info('First ultrasonic message received!')
            self.get_logger().info(f'  Left: {msg.left} mm')
            self.get_logger().info(f'  Mid: {msg.mid} mm')
            self.get_logger().info(f'  Right: {msg.right} mm')
            self.get_logger().info('=' * 60)

        # Log every 50 messages
        if self.message_count % 50 == 0:
            self.get_logger().info(
                f'Received {self.message_count} messages - '
                f'Latest: L={msg.left}, M={msg.mid}, R={msg.right}'
            )

    def publish_markers(self):
        """Publish visualization markers."""
        # Debug: Log every 100 publish attempts
        self.publish_count += 1
        if self.publish_count % 100 == 0:
            if self.latest_ultrasonic_data is None:
                self.get_logger().warn(
                    f'publish_markers called {self.publish_count} times, '
                    f'but NO ultrasonic data received yet!'
                )
            else:
                self.get_logger().info(
                    f'Publishing markers #{self.publish_count} - '
                    f'Data available: {self.message_count} messages received'
                )

        if self.latest_ultrasonic_data is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Get sensor readings
        sensors = {
            'left': self.latest_ultrasonic_data.left,
            'mid': self.latest_ultrasonic_data.mid,
            'right': self.latest_ultrasonic_data.right,
        }

        # Debug: Log sensor values for first publish
        if self.publish_count == 1:
            self.get_logger().info('=' * 60)
            self.get_logger().info('Starting to create markers...')
            self.get_logger().info(f'Sensor readings: L={sensors["left"]}, M={sensors["mid"]}, R={sensors["right"]}')

        # Create markers for each sensor
        for sensor_name, sensor_value in sensors.items():
            angle = self.sensor_angles[sensor_name]
            sensor_pos = self.sensor_positions[sensor_name]

            # Convert sensor value to real distance (mm)
            distance_mm = sensor_value * self.distance_scale

            # Convert to meters for RViz
            distance_m = distance_mm / 1000.0
            max_range_m = self.max_range / 1000.0
            min_range_m = self.min_range / 1000.0

            # Debug: Log conversion for first publish
            if self.publish_count == 1:
                self.get_logger().info(
                    f'{sensor_name}: value={sensor_value}, '
                    f'distance={distance_mm:.1f}mm ({distance_m:.3f}m), '
                    f'range=[{min_range_m:.3f}, {max_range_m:.3f}]m'
                )

            # Create cone marker to show sensor field of view
            cone_marker = self.create_cone_marker(
                marker_id,
                angle,
                max_range_m,
                sensor_name,
                sensor_pos
            )
            marker_array.markers.append(cone_marker)
            marker_id += 1

            # Create obstacle marker if in valid range
            if min_range_m <= distance_m <= max_range_m:
                obstacle_marker = self.create_obstacle_marker(
                    marker_id,
                    angle,
                    distance_m,
                    sensor_name,
                    sensor_pos
                )
                marker_array.markers.append(obstacle_marker)
                marker_id += 1

                if self.publish_count == 1:
                    self.get_logger().info(f'{sensor_name}: Created obstacle marker at {distance_m:.3f}m')
            else:
                if self.publish_count == 1:
                    self.get_logger().info(f'{sensor_name}: Distance out of range, no obstacle marker')

        # Publish marker array
        self.marker_pub.publish(marker_array)

        # Debug: Log first successful publish
        if self.publish_count == 1:
            self.get_logger().info(f'Total markers created: {len(marker_array.markers)}')
            self.get_logger().info('Marker array published!')

            # Check if anyone is listening
            sub_count = self.marker_pub.get_subscription_count()
            if sub_count == 0:
                self.get_logger().warn(
                    f'WARNING: Publishing to {self.marker_topic} but NO subscribers detected! '
                    f'Make sure RViz is running and subscribed to this topic.'
                )
            else:
                self.get_logger().info(f'Number of subscribers: {sub_count}')
            self.get_logger().info('=' * 60)

    def create_cone_marker(self, marker_id: int, angle: float,
                          max_range: float, sensor_name: str, sensor_pos: dict) -> Marker:
        """Create a cone-shaped marker to represent sensor field of view."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"ultrasonic_cone_{sensor_name}"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set marker lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)

        # Set color
        marker.color.r = self.cone_color['r']
        marker.color.g = self.cone_color['g']
        marker.color.b = self.cone_color['b']
        marker.color.a = self.cone_color['a']

        # Set scale (line width)
        marker.scale.x = 0.01

        # Create cone shape (opening angle of ~30 degrees)
        cone_angle = math.radians(15.0)  # Half angle

        # Start point (sensor position relative to base_link)
        p0 = Point()
        p0.x = sensor_pos['x']
        p0.y = sensor_pos['y']
        p0.z = 0.0

        # Left edge of cone
        p1 = Point()
        p1.x = sensor_pos['x'] + max_range * math.cos(angle + cone_angle)
        p1.y = sensor_pos['y'] + max_range * math.sin(angle + cone_angle)
        p1.z = 0.0

        # Right edge of cone
        p2 = Point()
        p2.x = sensor_pos['x'] + max_range * math.cos(angle - cone_angle)
        p2.y = sensor_pos['y'] + max_range * math.sin(angle - cone_angle)
        p2.z = 0.0

        # Create cone outline
        marker.points = [p0, p1, p2, p0]

        return marker

    def create_obstacle_marker(self, marker_id: int, angle: float,
                               distance: float, sensor_name: str, sensor_pos: dict) -> Marker:
        """Create a sphere marker to represent detected obstacle."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"ultrasonic_obstacle_{sensor_name}"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set marker lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)

        # Set position (at detected distance and angle, relative to sensor position)
        marker.pose.position.x = sensor_pos['x'] + distance * math.cos(angle)
        marker.pose.position.y = sensor_pos['y'] + distance * math.sin(angle)
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        # Set scale (sphere size)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set color
        marker.color.r = self.obstacle_color['r']
        marker.color.g = self.obstacle_color['g']
        marker.color.b = self.obstacle_color['b']
        marker.color.a = self.obstacle_color['a']

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicVizNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
