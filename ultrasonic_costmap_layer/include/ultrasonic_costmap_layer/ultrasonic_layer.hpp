/*
 * Ultrasonic Costmap Layer Plugin
 * Subscribes to ultrasonic sensor data and creates arc-shaped costmap obstacles
 */

 #ifndef ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
 #define ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
 
 #include <memory>
 #include <string>
 #include <vector>
 #include <mutex>
 #include <list>
 #include <cmath>
 #include <limits>
 
 #include "rclcpp/rclcpp.hpp"
 #include "nav2_costmap_2d/costmap_layer.hpp"
 #include "nav2_costmap_2d/layered_costmap.hpp"
 #include "car_chassis/msg/ultrasonic.hpp"
 #include <tf2/time.h>  // for tf2::Duration
 
 namespace ultrasonic_costmap_layer
 {
 
 class UltrasonicLayer : public nav2_costmap_2d::CostmapLayer
 {
 public:
   UltrasonicLayer();
   virtual ~UltrasonicLayer();
 
   void onInitialize() override;
 
   void updateBounds(
     double robot_x, double robot_y, double robot_yaw,
     double * min_x, double * min_y, double * max_x, double * max_y) override;
 
   void updateCosts(
     nav2_costmap_2d::Costmap2D & master_grid,
     int min_i, int min_j, int max_i, int max_j) override;
 
   void reset() override;
   void deactivate() override;
   void activate() override;
 
   bool isClearable() override { return true; }
 
 private:
   void bufferIncomingUltrasonicMsg(
     const car_chassis::msg::Ultrasonic::SharedPtr msg);
 
   void updateCostmap(double robot_x, double robot_y, double robot_yaw);
 
   void processSingleSensor(
     int distance_mm, double sensor_angle, const rclcpp::Time & stamp,
     double robot_x, double robot_y, double robot_yaw);
 
   void updateCostmapWithSensor(
     double distance, double sensor_angle, const rclcpp::Time & stamp,
     bool clear_sensor_cone,
     double robot_x, double robot_y, double robot_yaw,
     bool force_ray_clear_to_max);
 
   void update_cell(
     double ox, double oy, double ot, double r,
     double nx, double ny, bool clear);
 
   // Sensor model helpers
   double gamma(double theta);
   double delta(double phi);
   void get_deltas(double angle, double * dx, double * dy);
   double sensor_model(double r, double phi, double theta);
 
   bool isValidReading(double distance);
   void resetRange();
 
   // ROS2 subscriber
   rclcpp::Subscription<car_chassis::msg::Ultrasonic>::SharedPtr ultrasonic_sub_;
 
   // Thread-safety & buffering
   std::mutex ultrasonic_mutex_;
   std::list<car_chassis::msg::Ultrasonic> ultrasonic_msgs_buffer_;
   size_t buffered_readings_{0};
 
   // Layer state
   bool current_{true};
   bool was_reset_{false};
   unsigned char default_value_{0};
 
   // Parameters
   bool enabled_{true};
   std::string ultrasonic_topic_;
   std::string global_frame_;
 
   // Sensor configuration (angles in radians)
   double sensor_angle_left_{0.0};
   double sensor_angle_mid_{0.0};
   double sensor_angle_right_{0.0};
 
   // Range limits (meters)
   double min_range_{0.05};
   double max_range_{4.0};
 
   // Distance scale factor
   double distance_scale_{1.0};
 
   // Field of view per sensor (radians)
   double sensor_fov_{30.0 * M_PI / 180.0};
 
   // Probabilistic model & thresholds
   double phi_v_{1.2};
   double inflate_cone_{1.0};
   double clear_threshold_{0.2};
   double mark_threshold_{0.8};
   bool clear_on_max_reading_{false};
 
   // Ray clearing
   bool enable_ray_clear_{true};
   double ray_clear_margin_{0.10};    // meters
   int ray_clear_radius_cells_{0};    // 可保留为 0（单像素射线）；如需“管径清除”，可在 cpp 中扩展
 
   // Update bounds
   double min_x_{0.0}, min_y_{0.0}, max_x_{0.0}, max_y_{0.0};
 
   // Timeout for sensor data
   double no_readings_timeout_{2.0};
   rclcpp::Time last_reading_time_{0};
 
   // Transform tolerance
   tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};
 
   // Cached
   double max_angle_{0.0};  // half FOV in radians
 
   // NEW: per-sensor translation (meters) in base_link frame
   double sensor_left_tx_{0.0},  sensor_left_ty_{0.0};   // NEW
   double sensor_mid_tx_{0.0},   sensor_mid_ty_{0.0};    // NEW
   double sensor_right_tx_{0.0}, sensor_right_ty_{0.0};  // NEW
 };
 
 }  // namespace ultrasonic_costmap_layer
 
 #endif  // ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
 