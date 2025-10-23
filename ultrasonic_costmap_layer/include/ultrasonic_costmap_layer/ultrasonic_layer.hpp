/*
 * Copyright (c) 2024
 * All rights reserved.
 *
 * Ultrasonic Costmap Layer Plugin
 * Subscribes to ultrasonic sensor data and creates arc-shaped costmap obstacles
 */

#ifndef ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
#define ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "car_chassis/msg/ultrasonic.hpp"

namespace ultrasonic_costmap_layer
{

/**
 * @class UltrasonicLayer
 * @brief Costmap layer that creates arc-shaped obstacles from ultrasonic sensor data
 */
class UltrasonicLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief Constructor
   */
  UltrasonicLayer();

  /**
   * @brief Destructor
   */
  virtual ~UltrasonicLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_i X min map coord of the window to update
   * @param min_j Y min map coord of the window to update
   * @param max_i X max map coord of the window to update
   * @param max_j Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Reset this costmap
   */
  virtual void reset() override;

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate() override;

  /**
   * @brief Activate the layer
   */
  virtual void activate() override;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() override { return true; }

private:
  /**
   * @brief Callback for ultrasonic sensor data
   */
  void ultrasonicCallback(const car_chassis::msg::Ultrasonic::SharedPtr msg);

  /**
   * @brief Update costmap with sensor data
   */
  void updateCostmapWithUltrasonic();

  /**
   * @brief Draw an arc-shaped obstacle for a single sensor
   * @param sensor_angle Angle of the sensor (radians)
   * @param distance Distance to obstacle (meters)
   * @param robot_x Robot X position
   * @param robot_y Robot Y position
   * @param robot_yaw Robot orientation
   */
  void drawArcObstacle(
    double sensor_angle, double distance,
    double robot_x, double robot_y, double robot_yaw);

  /**
   * @brief Check if a distance reading is valid
   */
  bool isValidReading(double distance);

  /**
   * @brief Reset the update bounds
   */
  void resetRange();

  // ROS2 subscriber
  rclcpp::Subscription<car_chassis::msg::Ultrasonic>::SharedPtr ultrasonic_sub_;

  // Mutex for thread-safe access to sensor data
  std::mutex data_mutex_;

  // Latest ultrasonic data
  car_chassis::msg::Ultrasonic::SharedPtr latest_ultrasonic_data_;
  bool has_data_;

  // Parameters
  std::string ultrasonic_topic_;
  std::string global_frame_;

  // Sensor configuration (angles in radians)
  double sensor_angle_left_;
  double sensor_angle_mid_;
  double sensor_angle_right_;

  // Range limits (in meters)
  double min_range_;
  double max_range_;

  // Distance scale factor
  double distance_scale_;

  // Field of view for each sensor (radians)
  double sensor_fov_;

  // Arc thickness (meters)
  double arc_thickness_;

  // Obstacle cost value
  unsigned char obstacle_cost_;

  // Update bounds
  double min_x_, min_y_, max_x_, max_y_;

  // Timeout for sensor data
  double no_readings_timeout_;
  rclcpp::Time last_reading_time_;

  // Transform tolerance
  tf2::Duration transform_tolerance_;
};

}  // namespace ultrasonic_costmap_layer

#endif  // ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
