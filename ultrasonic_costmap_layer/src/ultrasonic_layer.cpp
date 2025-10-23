/*
 * Copyright (c) 2024
 * All rights reserved.
 *
 * Ultrasonic Costmap Layer Plugin Implementation
 */

#include "ultrasonic_costmap_layer/ultrasonic_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(ultrasonic_costmap_layer::UltrasonicLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace ultrasonic_costmap_layer
{

UltrasonicLayer::UltrasonicLayer()
: has_data_(false),
  last_reading_time_(rclcpp::Time(0))
{
}

UltrasonicLayer::~UltrasonicLayer()
{
}

void UltrasonicLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Initialize layer
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  // Declare and get parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("ultrasonic_topic", rclcpp::ParameterValue("/ultrasonic"));
  node->get_parameter(name_ + "." + "ultrasonic_topic", ultrasonic_topic_);

  declareParameter("sensor_angle_left", rclcpp::ParameterValue(45.0));
  declareParameter("sensor_angle_mid", rclcpp::ParameterValue(0.0));
  declareParameter("sensor_angle_right", rclcpp::ParameterValue(-45.0));

  double angle_left_deg, angle_mid_deg, angle_right_deg;
  node->get_parameter(name_ + "." + "sensor_angle_left", angle_left_deg);
  node->get_parameter(name_ + "." + "sensor_angle_mid", angle_mid_deg);
  node->get_parameter(name_ + "." + "sensor_angle_right", angle_right_deg);

  // Convert to radians
  sensor_angle_left_ = angle_left_deg * M_PI / 180.0;
  sensor_angle_mid_ = angle_mid_deg * M_PI / 180.0;
  sensor_angle_right_ = angle_right_deg * M_PI / 180.0;

  declareParameter("min_range", rclcpp::ParameterValue(50.0));
  declareParameter("max_range", rclcpp::ParameterValue(4000.0));
  node->get_parameter(name_ + "." + "min_range", min_range_);
  node->get_parameter(name_ + "." + "max_range", max_range_);

  // Convert from mm to meters
  min_range_ = min_range_ / 1000.0;
  max_range_ = max_range_ / 1000.0;

  declareParameter("distance_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "distance_scale", distance_scale_);

  declareParameter("sensor_fov", rclcpp::ParameterValue(30.0));
  double fov_deg;
  node->get_parameter(name_ + "." + "sensor_fov", fov_deg);
  sensor_fov_ = fov_deg * M_PI / 180.0;

  declareParameter("arc_thickness", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + "." + "arc_thickness", arc_thickness_);

  declareParameter("no_readings_timeout", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);

  // Transform tolerance
  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // Cost value for obstacles
  obstacle_cost_ = LETHAL_OBSTACLE;

  // Get global frame
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create subscriber with compatible QoS
  // Use RELIABLE + TRANSIENT_LOCAL to match rosbag recordings
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

  ultrasonic_sub_ = node->create_subscription<car_chassis::msg::Ultrasonic>(
    ultrasonic_topic_,
    qos_profile,
    std::bind(&UltrasonicLayer::ultrasonicCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "UltrasonicLayer: Initialized. Subscribing to %s",
    ultrasonic_topic_.c_str());

  RCLCPP_INFO(
    logger_,
    "Sensor angles (deg): left=%.1f, mid=%.1f, right=%.1f",
    angle_left_deg, angle_mid_deg, angle_right_deg);

  RCLCPP_INFO(
    logger_,
    "Range: %.3f - %.3f m, scale: %.2f, FOV: %.1f deg",
    min_range_, max_range_, distance_scale_, fov_deg);

  // Initialize last_reading_time_ with the correct clock to avoid time source mismatch
  last_reading_time_ = clock_->now();

  resetRange();
}

void UltrasonicLayer::ultrasonicCallback(
  const car_chassis::msg::Ultrasonic::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_ultrasonic_data_ = msg;
  has_data_ = true;
  last_reading_time_ = clock_->now();
  RCLCPP_INFO(
    logger_,
    "received ultrasonic data: left=%d mm, mid=%d mm, right=%d mm",
    msg->left, msg->mid, msg->right);
}

void UltrasonicLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_x;  // Unused - position handled by costmap transform
  (void)robot_y;  // Unused - position handled by costmap transform
  (void)robot_yaw;  // Unused - orientation handled by costmap transform

  if (!enabled_) {
    return;
  }

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Check for timeout
  if (no_readings_timeout_ > 0.0) {
    if ((clock_->now() - last_reading_time_).seconds() > no_readings_timeout_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "No ultrasonic readings for %.2f seconds",
        (clock_->now() - last_reading_time_).seconds());
      current_ = false;
      return;
    }
  }

  // Update costmap with latest data
  updateCostmapWithUltrasonic();

  // Update bounds
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  current_ = true;
}

void UltrasonicLayer::updateCostmapWithUltrasonic()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!has_data_ || !latest_ultrasonic_data_) {
    return;
  }

  resetRange();

  // Get robot pose (we're already in the correct frame from updateBounds)
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;

  // Convert sensor readings to meters
  double left_dist = latest_ultrasonic_data_->left * distance_scale_ / 1000.0;
  double mid_dist = latest_ultrasonic_data_->mid * distance_scale_ / 1000.0;
  double right_dist = latest_ultrasonic_data_->right * distance_scale_ / 1000.0;

  // Draw arcs for each sensor if reading is valid
  if (isValidReading(left_dist)) {
    drawArcObstacle(sensor_angle_left_, left_dist, robot_x, robot_y, robot_yaw);
  }

  if (isValidReading(mid_dist)) {
    drawArcObstacle(sensor_angle_mid_, mid_dist, robot_x, robot_y, robot_yaw);
  }

  if (isValidReading(right_dist)) {
    drawArcObstacle(sensor_angle_right_, right_dist, robot_x, robot_y, robot_yaw);
  }
}

bool UltrasonicLayer::isValidReading(double distance)
{
  return distance >= min_range_ && distance <= max_range_;
}

void UltrasonicLayer::drawArcObstacle(
  double sensor_angle, double distance,
  double robot_x, double robot_y, double robot_yaw)
{
  // Calculate the center point of the arc (obstacle position)
  double total_angle = robot_yaw + sensor_angle;
  double obstacle_x = robot_x + distance * cos(total_angle);
  double obstacle_y = robot_y + distance * sin(total_angle);

  // Update bounds to include the arc area
  touch(obstacle_x, obstacle_y, &min_x_, &min_y_, &max_x_, &max_y_);

  // Calculate arc parameters
  double half_fov = sensor_fov_ / 2.0;
  double arc_inner_radius = distance - arc_thickness_ / 2.0;
  double arc_outer_radius = distance + arc_thickness_ / 2.0;

  // Sample points along the arc
  int num_arc_samples = 20;  // Number of points to sample along the arc

  for (int i = 0; i <= num_arc_samples; i++) {
    // Angle within the sensor's FOV
    double fov_angle = -half_fov + (2.0 * half_fov * i) / num_arc_samples;
    double sample_angle = total_angle + fov_angle;

    // Draw points at different radii to create thickness
    int num_radial_samples = std::max(3, static_cast<int>(arc_thickness_ / resolution_));

    for (int r = 0; r <= num_radial_samples; r++) {
      double radius = arc_inner_radius +
                     (arc_outer_radius - arc_inner_radius) * r / num_radial_samples;

      if (radius < 0) continue;

      double px = robot_x + radius * cos(sample_angle);
      double py = robot_y + radius * sin(sample_angle);

      unsigned int cell_x, cell_y;
      if (worldToMap(px, py, cell_x, cell_y)) {
        setCost(cell_x, cell_y, obstacle_cost_);
      }
    }
  }
}

void UltrasonicLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // Update master costmap with our obstacles
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char cost = costmap_[it];

      if (cost == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      // Only update if our cost is higher
      if (old_cost == NO_INFORMATION || old_cost < cost) {
        master_array[it] = cost;
      }

      it++;
    }
  }
}

void UltrasonicLayer::resetRange()
{
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = -std::numeric_limits<double>::max();
  max_y_ = -std::numeric_limits<double>::max();
}

void UltrasonicLayer::reset()
{
  RCLCPP_DEBUG(logger_, "Resetting ultrasonic costmap layer...");
  deactivate();
  resetMaps();
  resetRange();
  current_ = true;
  activate();
}

void UltrasonicLayer::deactivate()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  has_data_ = false;
}

void UltrasonicLayer::activate()
{
  // Nothing special needed on activation
}

}  // namespace ultrasonic_costmap_layer
