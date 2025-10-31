/*
*
* Ultrasonic Costmap Layer - Implementation
*/

#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <limits>
#include <string>
#include <vector>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "ultrasonic_costmap_layer/ultrasonic_layer.hpp"

PLUGINLIB_EXPORT_CLASS(ultrasonic_costmap_layer::UltrasonicLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace ultrasonic_costmap_layer
{

// ---------- helpers (from RangeSensorLayer) ----------

static inline unsigned char to_cost(double p) { return static_cast<unsigned char>(std::round(p * 252)); }
static inline double to_prob(unsigned char c) { return static_cast<double>(c) / 252.0; }

static inline int orient2d(int ax, int ay, int bx, int by, int cx, int cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static inline double area(int ax, int ay, int bx, int by, int cx, int cy)
{
  return std::fabs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0;
}

// ---------- ctor / dtor ----------

UltrasonicLayer::UltrasonicLayer()
: buffered_readings_(0),   // 与 hpp 声明顺序一致，避免 -Wreorder
  current_(true),
  was_reset_(false),
  last_reading_time_(rclcpp::Time(0))
{
}

UltrasonicLayer::~UltrasonicLayer() = default;

// ---------- onInitialize ----------

void UltrasonicLayer::onInitialize()
{
  current_ = true;
  was_reset_ = false;
  buffered_readings_ = 0;
  last_reading_time_ = clock_->now();
  default_value_ = to_cost(0.5);

  matchSize();
  setDefaultValue(default_value_);   // 确保内部 costmap_ 初始化为 0.5
  resetMaps();
  resetRange();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("ultrasonic_topic", rclcpp::ParameterValue("/ultrasonic"));
  node->get_parameter(name_ + "." + "ultrasonic_topic", ultrasonic_topic_);

  declareParameter("sensor_angle_left", rclcpp::ParameterValue(45.0));
  declareParameter("sensor_angle_mid", rclcpp::ParameterValue(0.0));
  declareParameter("sensor_angle_right", rclcpp::ParameterValue(-45.0));

  double angle_left_deg{45.0}, angle_mid_deg{0.0}, angle_right_deg{-45.0};
  node->get_parameter(name_ + "." + "sensor_angle_left", angle_left_deg);
  node->get_parameter(name_ + "." + "sensor_angle_mid", angle_mid_deg);
  node->get_parameter(name_ + "." + "sensor_angle_right", angle_right_deg);

  sensor_angle_left_  = angle_left_deg  * M_PI / 180.0;
  sensor_angle_mid_   = angle_mid_deg   * M_PI / 180.0;
  sensor_angle_right_ = angle_right_deg * M_PI / 180.0;

  declareParameter("min_range", rclcpp::ParameterValue(50.0));
  declareParameter("max_range", rclcpp::ParameterValue(4000.0));
  double min_range_mm{50.0}, max_range_mm{4000.0};
  node->get_parameter(name_ + "." + "min_range", min_range_mm);
  node->get_parameter(name_ + "." + "max_range", max_range_mm);
  min_range_ = min_range_mm / 1000.0;
  max_range_ = max_range_mm / 1000.0;

  declareParameter("distance_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "distance_scale", distance_scale_);

  declareParameter("sensor_fov", rclcpp::ParameterValue(30.0));
  double fov_deg{30.0};
  node->get_parameter(name_ + "." + "sensor_fov", fov_deg);
  sensor_fov_ = fov_deg * M_PI / 180.0;

  declareParameter("no_readings_timeout", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);

  declareParameter("phi", rclcpp::ParameterValue(1.2));
  node->get_parameter(name_ + "." + "phi", phi_v_);

  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);

  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);

  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);

  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  // Ray clear params
  declareParameter("enable_ray_clear", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enable_ray_clear", enable_ray_clear_);

  declareParameter("ray_clear_margin", rclcpp::ParameterValue(0.10));
  node->get_parameter(name_ + "." + "ray_clear_margin", ray_clear_margin_);

  declareParameter("ray_clear_radius_cells", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + "." + "ray_clear_radius_cells", ray_clear_radius_cells_);

  // NEW: per-sensor translation (meters) in base_link frame
  declareParameter("sensor_left_tx",  rclcpp::ParameterValue(0.0));  // NEW
  declareParameter("sensor_left_ty",  rclcpp::ParameterValue(0.0));  // NEW
  declareParameter("sensor_mid_tx",   rclcpp::ParameterValue(0.0));  // NEW
  declareParameter("sensor_mid_ty",   rclcpp::ParameterValue(0.0));  // NEW
  declareParameter("sensor_right_tx", rclcpp::ParameterValue(0.0));  // NEW
  declareParameter("sensor_right_ty", rclcpp::ParameterValue(0.0));  // NEW

  node->get_parameter(name_ + "." + "sensor_left_tx",  sensor_left_tx_);   // NEW
  node->get_parameter(name_ + "." + "sensor_left_ty",  sensor_left_ty_);   // NEW
  node->get_parameter(name_ + "." + "sensor_mid_tx",   sensor_mid_tx_);    // NEW
  node->get_parameter(name_ + "." + "sensor_mid_ty",   sensor_mid_ty_);    // NEW
  node->get_parameter(name_ + "." + "sensor_right_tx", sensor_right_tx_);  // NEW
  node->get_parameter(name_ + "." + "sensor_right_ty", sensor_right_ty_);  // NEW

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  ultrasonic_sub_ = node->create_subscription<car_chassis::msg::Ultrasonic>(
    ultrasonic_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&UltrasonicLayer::bufferIncomingUltrasonicMsg, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "UltrasonicLayer: Initialized and subscribed to %s", ultrasonic_topic_.c_str());
  RCLCPP_INFO(logger_, "Sensor angles (deg): L=%.1f, M=%.1f, R=%.1f",
              angle_left_deg, angle_mid_deg, angle_right_deg);
  RCLCPP_INFO(logger_, "Range: %.3f-%.3f m, FOV: %.1f deg, phi: %.2f, ray_clear=%s, margin=%.2f m",
              min_range_, max_range_, fov_deg, phi_v_,
              enable_ray_clear_ ? "on" : "off", ray_clear_margin_);
  RCLCPP_INFO(logger_, "Sensor offsets (m): L(%.3f,%.3f) M(%.3f,%.3f) R(%.3f,%.3f)",
              sensor_left_tx_, sensor_left_ty_, sensor_mid_tx_, sensor_mid_ty_, sensor_right_tx_, sensor_right_ty_);
}

// ---------- sensor model ----------

double UltrasonicLayer::gamma(double theta)
{
  if (std::fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    return 1 - std::pow(theta / max_angle_, 2);
  }
}

double UltrasonicLayer::delta(double phi)
{
  return 1 - (1 + std::tanh(2 * (phi - phi_v_))) / 2;
}

void UltrasonicLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = std::tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    *dx = resolution_ / ta;
  }
  *dx = std::copysign(*dx, std::cos(angle));
  *dy = std::copysign(resolution_, std::sin(angle));
}

double UltrasonicLayer::sensor_model(double r, double phi, double theta)
{
  double lbda = delta(phi) * gamma(theta);
  double delta_val = resolution_;

  if (phi >= 0.0 && phi < r - 2 * delta_val * r) {
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta_val * r) {
    return lbda * 0.5 * std::pow((phi - (r - 2 * delta_val * r)) / (delta_val * r), 2) +
           (1 - lbda) * 0.5;
  } else if (phi < r + delta_val * r) {
    double J = (r - phi) / (delta_val * r);
    return lbda * ((1 - (0.5) * std::pow(J, 2)) - 0.5) + 0.5;
  } else {
    return 0.5;
  }
}

// ---------- message handling ----------

void UltrasonicLayer::bufferIncomingUltrasonicMsg(
  const car_chassis::msg::Ultrasonic::SharedPtr msg)
{
  static int print_cnt = 0;
  if (print_cnt < 5) {
    RCLCPP_INFO(logger_, "Ultrasonic rx: L=%d mm M=%d mm R=%d mm", msg->left, msg->mid, msg->right);
    ++print_cnt;
  }
  std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
  ultrasonic_msgs_buffer_.push_back(*msg);
}


void UltrasonicLayer::updateCostmap(double robot_x, double robot_y, double robot_yaw)
{
  std::list<car_chassis::msg::Ultrasonic> ultrasonic_buffer_copy;
  {
    std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
    ultrasonic_msgs_buffer_.swap(ultrasonic_buffer_copy);
  }

  for (auto & msg : ultrasonic_buffer_copy) {
    rclcpp::Time timestamp = clock_->now();  // prefer header.stamp if available
    processSingleSensor(msg.left,  sensor_angle_left_,  timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.mid,   sensor_angle_mid_,   timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.right, sensor_angle_right_, timestamp, robot_x, robot_y, robot_yaw);
  }
}

void UltrasonicLayer::processSingleSensor(
  int distance_mm,
  double sensor_angle,
  const rclcpp::Time & timestamp,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  // 1) 距离（mm->m）
  double distance = distance_mm * distance_scale_ / 1000.0;
  if (!isValidReading(distance)) {
    return;
  }

  // 2) 是否只做“最大量程射线清除”
  bool clear_sensor_cone = false;
  bool force_ray_clear_to_max = false;
  if (distance >= max_range_ - 1e-2 && clear_on_max_reading_) {
    force_ray_clear_to_max = true;
  }

  // 3) NEW: 选择该传感器的安装位移（base_link 坐标系）
  double tx_b = 0.0, ty_b = 0.0;  // base_link frame
  // 用角度指针匹配（这里传入的就是已配置的三个角度变量）
  if (sensor_angle == sensor_angle_left_) {
    tx_b = sensor_left_tx_;  ty_b = sensor_left_ty_;
  } else if (sensor_angle == sensor_angle_mid_) {
    tx_b = sensor_mid_tx_;   ty_b = sensor_mid_ty_;
  } else if (sensor_angle == sensor_angle_right_) {
    tx_b = sensor_right_tx_; ty_b = sensor_right_ty_;
  }

  // 4) NEW: 旋转到全局，得到传感器原点（全局坐标系）
  const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
  const double sensor_ox = robot_x + c * tx_b - s * ty_b;  // global x
  const double sensor_oy = robot_y + s * tx_b + c * ty_b;  // global y

  // 5) 用“传感器真实原点”进行更新
  updateCostmapWithSensor(
    distance, sensor_angle, timestamp, clear_sensor_cone,
    sensor_ox, sensor_oy, robot_yaw, force_ray_clear_to_max);
}

void UltrasonicLayer::updateCostmapWithSensor(
  double distance,
  double sensor_angle,
  const rclcpp::Time & /*timestamp*/,
  bool clear_sensor_cone,
  double robot_x,
  double robot_y,
  double robot_yaw,
  bool force_ray_clear_to_max)
{
  max_angle_ = sensor_fov_ / 2.0;

  // 注意：这里的 (robot_x, robot_y) 已是“该传感器”的原点（全局）
  const double ox = robot_x;
  const double oy = robot_y;

  const double total_angle = robot_yaw + sensor_angle;

  const double tx = ox + distance * std::cos(total_angle);
  const double ty = oy + distance * std::sin(total_angle);

  const double dx = tx - ox;
  const double dy = ty - oy;
  const double theta = std::atan2(dy, dx);
  const double d = std::hypot(dx, dy);

  // ---------- Ray clearing ----------
  const double effective_r = force_ray_clear_to_max ? max_range_ : distance;

  if (!clear_sensor_cone && enable_ray_clear_) {
    double step_x, step_y;
    get_deltas(total_angle, &step_x, &step_y);

    const double target = std::max(0.0, effective_r - ray_clear_margin_);

    double wx = ox, wy = oy;
    double travelled = 0.0;

    while (travelled + 0.5 * resolution_ < target) {
      // 如需“管径清除”，可在此扩展邻域；当前保留单像素射线
      update_cell(ox, oy, total_angle, effective_r, wx, wy, /*clear=*/true);
      wx += step_x;
      wy += step_y;
      travelled = std::hypot(wx - ox, wy - oy);
    }
  }
  // ---------- End ray clearing ----------

  // Init update bounds
  int bx0, by0, bx1, by1;
  int Ox, Oy, Ax, Ay, Bx, By;

  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  double mx, my;

  // left border
  mx = ox + std::cos(theta - max_angle_) * d * 1.2;
  my = oy + std::sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  RCLCPP_INFO(logger_, "US cone: r=%.2f ang=%.2f bbox=[%d,%d]-[%d,%d]",
             distance, sensor_angle, bx0, by0, bx1, by1);

  // right border
  mx = ox + std::cos(theta + max_angle_) * d * 1.2;
  my = oy + std::sin(theta + max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // ---- clamp bounds (off-by-one fix) ----
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_) - 1, bx1);
  by1 = std::min(static_cast<int>(size_y_) - 1, by1);
  if (bx0 > bx1 || by0 > by1) {
    return;
  }

  // iterate cells
  for (unsigned int x = static_cast<unsigned int>(bx0); x <= static_cast<unsigned int>(bx1); x++) {
    for (unsigned int y = static_cast<unsigned int>(by0); y <= static_cast<unsigned int>(by1); y++) {
      bool update_xy_cell = true;

      {
        int w0 = orient2d(Ax, Ay, Bx, By, static_cast<int>(x), static_cast<int>(y));
        int w1 = orient2d(Bx, By, Ox, Oy, static_cast<int>(x), static_cast<int>(y));
        int w2 = orient2d(Ox, Oy, Ax, Ay, static_cast<int>(x), static_cast<int>(y));
        float thr = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = (w0 >= thr) && (w1 >= thr) && (w2 >= thr);
      }

      if (update_xy_cell) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, distance, wx, wy, clear_sensor_cone);
      }
    }
  }

  buffered_readings_++;
  last_reading_time_ = clock_->now();
}

void UltrasonicLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    double dx = nx - ox;
    double dy = ny - oy;
    double theta = std::atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = std::hypot(dx, dy);

    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }

    double prior = to_prob(getCost(x, y));

    // 防御：避免 0/0
    double denom = sensor * prior + (1 - sensor) * (1 - prior);
    double new_prob = 0.5;
    if (clear) {
      new_prob = 0.0;
    } else if (denom > 1e-12) {
      new_prob = (sensor * prior) / denom;
    } else {
      new_prob = 0.5;
    }

    unsigned char c = to_cost(new_prob);
    setCost(x, y, c);
  }
}

// ---------- Layer interface ----------

void UltrasonicLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  updateCostmap(robot_x, robot_y, robot_yaw);

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();

  if (!enabled_) {
    current_ = true;
    return;
  }

  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
        (clock_->now() - last_reading_time_).seconds() > no_readings_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "No ultrasonic readings for %.2f seconds",
        (clock_->now() - last_reading_time_).seconds());
      current_ = false;
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

  // clamp window to grid
  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  min_i = std::max(0, std::min(min_i, size_x));
  max_i = std::max(0, std::min(max_i, size_x));
  min_j = std::max(0, std::min(min_j, size_y));
  max_j = std::max(0, std::min(max_j, size_y));
  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_);
  unsigned char mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = static_cast<unsigned int>(j) * span + static_cast<unsigned int>(min_i);
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;
      RCLCPP_INFO(logger_, "updateCosts: i=[%d,%d) j=[%d,%d)", min_i, max_i, min_j, max_j);

      if (prob == NO_INFORMATION) {
        it++;
        continue;
      } else if (prob > mark) {
        current = LETHAL_OBSTACLE;
      } else if (prob < clear) {
        current = FREE_SPACE;
      } else {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      // SAFE clearing & only-increase marking
      if (current == FREE_SPACE) {
        // 仅在 UNKNOWN 或已有 FREE 时下调，避免误清他层障碍/膨胀
        if (old_cost == NO_INFORMATION || old_cost == FREE_SPACE) {
          master_array[it] = FREE_SPACE;
        }
      } else { // LETHAL_OBSTACLE
        if (old_cost == NO_INFORMATION || old_cost < LETHAL_OBSTACLE) {
          master_array[it] = LETHAL_OBSTACLE;
        }
      }

      it++;
    }
  }

  buffered_readings_ = 0;

  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
}

void UltrasonicLayer::reset()
{
  RCLCPP_DEBUG(logger_, "Resetting ultrasonic layer...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

void UltrasonicLayer::deactivate()
{
  std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
  ultrasonic_msgs_buffer_.clear();
}

void UltrasonicLayer::activate()
{
  std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
  ultrasonic_msgs_buffer_.clear();
}

// ---------- utils ----------

void UltrasonicLayer::resetRange()
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

bool UltrasonicLayer::isValidReading(double distance)
{
  return distance >= min_range_ && distance <= max_range_;
}

}  // namespace ultrasonic_costmap_layer
