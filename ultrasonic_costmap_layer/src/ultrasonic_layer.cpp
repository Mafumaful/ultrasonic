/*
 * Copyright (c) 2024
 * Based on Nav2 RangeSensorLayer (David V. Lu!!, Bytes Robotics)
 *
 * Ultrasonic Costmap Layer - Implementation
 * 基于官方 RangeSensorLayer 结构，适配三传感器超声波消息
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

// 注册插件
PLUGINLIB_EXPORT_CLASS(ultrasonic_costmap_layer::UltrasonicLayer, nav2_costmap_2d::Layer)

// 使用官方常量
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace ultrasonic_costmap_layer
{

// ========== 静态辅助函数（来自官方RangeSensorLayer）==========

/**
 * 【官方实现】概率转代价值
 * 将 [0, 1] 概率映射到 [0, 252] 代价
 */
static inline unsigned char to_cost(double p)
{
  return static_cast<unsigned char>(round(p * 252));
}

/**
 * 【官方实现】代价值转概率
 * 将 [0, 252] 代价映射到 [0, 1] 概率
 */
static inline double to_prob(unsigned char c)
{
  return static_cast<double>(c) / 252.0;
}

/**
 * 【官方实现】计算三角形有向面积的2倍
 */
static inline int orient2d(int ax, int ay, int bx, int by, int cx, int cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/**
 * 【官方实现】计算三角形面积
 */
static inline double area(int ax, int ay, int bx, int by, int cx, int cy)
{
  return fabs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0;
}

// ========== 构造和析构 ==========

UltrasonicLayer::UltrasonicLayer()
: current_(true),
  was_reset_(false),
  buffered_readings_(0),
  last_reading_time_(rclcpp::Time(0))
{
}

UltrasonicLayer::~UltrasonicLayer()
{
}

// ========== 初始化 ==========

void UltrasonicLayer::onInitialize()
{
  // 【官方结构】初始化状态变量
  current_ = true;
  was_reset_ = false;
  buffered_readings_ = 0;
  last_reading_time_ = clock_->now();
  default_value_ = to_cost(0.5);  // 默认概率 0.5

  matchSize();
  resetRange();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ========== 你的参数 ==========

  // 启用标志
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // 超声波话题
  declareParameter("ultrasonic_topic", rclcpp::ParameterValue("/ultrasonic"));
  node->get_parameter(name_ + "." + "ultrasonic_topic", ultrasonic_topic_);

  // 传感器角度（度 → 弧度）
  declareParameter("sensor_angle_left", rclcpp::ParameterValue(45.0));
  declareParameter("sensor_angle_mid", rclcpp::ParameterValue(0.0));
  declareParameter("sensor_angle_right", rclcpp::ParameterValue(-45.0));

  double angle_left_deg, angle_mid_deg, angle_right_deg;
  node->get_parameter(name_ + "." + "sensor_angle_left", angle_left_deg);
  node->get_parameter(name_ + "." + "sensor_angle_mid", angle_mid_deg);
  node->get_parameter(name_ + "." + "sensor_angle_right", angle_right_deg);

  sensor_angle_left_ = angle_left_deg * M_PI / 180.0;
  sensor_angle_mid_ = angle_mid_deg * M_PI / 180.0;
  sensor_angle_right_ = angle_right_deg * M_PI / 180.0;

  // 距离范围（毫米，转换为米）
  declareParameter("min_range", rclcpp::ParameterValue(50.0));    // mm
  declareParameter("max_range", rclcpp::ParameterValue(4000.0));  // mm
  double min_range_mm, max_range_mm;
  node->get_parameter(name_ + "." + "min_range", min_range_mm);
  node->get_parameter(name_ + "." + "max_range", max_range_mm);

  min_range_ = min_range_mm / 1000.0;  // 转换为米
  max_range_ = max_range_mm / 1000.0;

  // 距离校准系数
  declareParameter("distance_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "distance_scale", distance_scale_);

  // 传感器视场角（度 → 弧度）
  declareParameter("sensor_fov", rclcpp::ParameterValue(30.0));
  double fov_deg;
  node->get_parameter(name_ + "." + "sensor_fov", fov_deg);
  sensor_fov_ = fov_deg * M_PI / 180.0;

  // 超时设置
  declareParameter("no_readings_timeout", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);

  // ========== 官方参数 ==========

  // phi: 传感器模型锐度参数
  declareParameter("phi", rclcpp::ParameterValue(1.2));
  node->get_parameter(name_ + "." + "phi", phi_v_);

  // inflate_cone: 锥形膨胀系数
  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);

  // 阈值参数
  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);

  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);

  // 最大距离清除
  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  // TF 容差
  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // 全局坐标系
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // ========== 创建订阅者 ==========
  // 使用 RELIABLE + VOLATILE 以匹配 ultrasonic_timestamp_converter 的发布者
  // SystemDefaultsQoS = RELIABLE + VOLATILE，与大多数节点兼容
  ultrasonic_sub_ = node->create_subscription<car_chassis::msg::Ultrasonic>(
    ultrasonic_topic_,
    rclcpp::SystemDefaultsQoS(),  // RELIABLE + VOLATILE
    std::bind(&UltrasonicLayer::bufferIncomingUltrasonicMsg, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "UltrasonicLayer: Initialized and subscribed to %s",
    ultrasonic_topic_.c_str());

  RCLCPP_INFO(
    logger_,
    "Sensor angles (deg): left=%.1f, mid=%.1f, right=%.1f",
    angle_left_deg, angle_mid_deg, angle_right_deg);

  RCLCPP_INFO(
    logger_,
    "Range: %.3f - %.3f m, FOV: %.1f deg, phi: %.2f",
    min_range_, max_range_, fov_deg, phi_v_);
}

// ========== 官方函数：传感器模型 ==========

/**
 * 【官方实现】角度权重函数
 * gamma(θ) = 1 - (θ / max_angle)^2
 */
double UltrasonicLayer::gamma(double theta)
{
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    return 1 - pow(theta / max_angle_, 2);
  }
}

/**
 * 【官方实现】距离衰减函数
 * delta(φ) = 1 - (1 + tanh(2*(φ - phi_v))) / 2
 */
double UltrasonicLayer::delta(double phi)
{
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

/**
 * 【官方实现】计算扫描线步长
 */
void UltrasonicLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    *dx = resolution_ / ta;
  }

  *dx = copysign(*dx, cos(angle));
  *dy = copysign(resolution_, sin(angle));
}

/**
 * 【官方实现】传感器概率模型
 * 分段函数，在测量距离附近概率最高
 */
double UltrasonicLayer::sensor_model(double r, double phi, double theta)
{
  double lbda = delta(phi) * gamma(theta);
  double delta_val = resolution_;

  if (phi >= 0.0 && phi < r - 2 * delta_val * r) {
    // 距离明显小于测量值：可能是自由空间
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta_val * r) {
    // 接近测量距离（前沿）：概率开始上升
    return lbda * 0.5 * pow((phi - (r - 2 * delta_val * r)) / (delta_val * r), 2) +
           (1 - lbda) * 0.5;
  } else if (phi < r + delta_val * r) {
    // 在测量距离附近：障碍物概率最高
    double J = (r - phi) / (delta_val * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    // 距离大于测量值：未知区域
    return 0.5;
  }
}

// ========== 消息处理 ==========

/**
 * 【官方模式】缓冲传入的超声波消息（线程安全）
 */
void UltrasonicLayer::bufferIncomingUltrasonicMsg(
  const car_chassis::msg::Ultrasonic::SharedPtr msg)
{
  // RCLCPP_INFO(
  //   logger_,
  //   "UltrasonicLayer: Received ultrasonic message: left=%d mm, mid=%d mm, right=%d mm",
  //   msg->left, msg->mid, msg->right);
  ultrasonic_mutex_.lock();
  ultrasonic_msgs_buffer_.push_back(*msg);
  ultrasonic_mutex_.unlock();
}

/**
 * @brief 【官方模式】处理缓冲区中的所有消息
 */
void UltrasonicLayer::updateCostmap(double robot_x, double robot_y, double robot_yaw)
{
  std::list<car_chassis::msg::Ultrasonic> ultrasonic_buffer_copy;

  // 线程安全地复制并清空缓冲区
  ultrasonic_mutex_.lock();
  ultrasonic_buffer_copy = std::list<car_chassis::msg::Ultrasonic>(ultrasonic_msgs_buffer_);
  ultrasonic_msgs_buffer_.clear();
  ultrasonic_mutex_.unlock();

  // 处理每条消息
  for (auto & msg : ultrasonic_buffer_copy) {
    // 从消息头获取时间戳（如果有的话）
    // 注意：你的消息可能没有 header，这里使用当前时间
    rclcpp::Time timestamp = clock_->now();

    // 处理三个传感器，传递机器人位姿
    processSingleSensor(msg.left, sensor_angle_left_, timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.mid, sensor_angle_mid_, timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.right, sensor_angle_right_, timestamp, robot_x, robot_y, robot_yaw);
  }
}

/**
 * @brief 处理单个传感器的数据
 */
void UltrasonicLayer::processSingleSensor(
  int distance_mm,
  double sensor_angle,
  const rclcpp::Time & timestamp,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  // 单位转换：毫米 → 米，应用校准系数
  double distance = distance_mm * distance_scale_ / 1000.0;

  // 验证距离范围
  if (!isValidReading(distance)) {
    return;
  }

  // 判断是否清除锥形区域
  bool clear_sensor_cone = false;
  if (distance >= max_range_ - 0.01 && clear_on_max_reading_) {
    clear_sensor_cone = true;
  }

  // 更新代价地图，传递机器人位姿
  updateCostmapWithSensor(distance, sensor_angle, timestamp, clear_sensor_cone,
                          robot_x, robot_y, robot_yaw);
}

/**
 * 【基于官方实现】更新代价地图
 * 参考 range_sensor_layer.cpp:288-402
 */
void UltrasonicLayer::updateCostmapWithSensor(
  double distance,
  double sensor_angle,
  const rclcpp::Time & timestamp,
  bool clear_sensor_cone,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  max_angle_ = sensor_fov_ / 2.0;  // 半视场角

  // 使用真实的机器人位置（全局坐标系）
  double ox = robot_x;
  double oy = robot_y;

  // 计算检测点位置（全局坐标系）
  // sensor_angle 是相对于 base_link 的角度，robot_yaw 是机器人在全局坐标系中的朝向
  double total_angle = robot_yaw + sensor_angle;
  double tx = ox + distance * cos(total_angle);
  double ty = oy + distance * sin(total_angle);

  // 计算目标属性
  double dx = tx - ox;
  double dy = ty - oy;
  double theta = atan2(dy, dx);
  double d = sqrt(dx * dx + dy * dy);

  // 【官方实现】初始化更新区域边界
  int bx0, by0, bx1, by1;
  int Ox, Oy, Ax, Ay, Bx, By;

  // 原点
  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // 标记目标点
  unsigned int aa, ab;
  if (worldToMap(tx, ty, aa, ab)) {
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;

  // 【官方实现】锥形左边界
  mx = ox + cos(theta - max_angle_) * d * 1.2;
  my = oy + sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // 【官方实现】锥形右边界
  mx = ox + cos(theta + max_angle_) * d * 1.2;
  my = oy + sin(theta + max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // 限制边界
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_), bx1);
  by1 = std::min(static_cast<int>(size_y_), by1);

  // 【官方实现】遍历更新区域
  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      bool update_xy_cell = true;

      // 【官方实现】锥形区域过滤
      if (inflate_cone_ < 1.0) {
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        float bcciath = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      // 【官方实现】更新栅格
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

/**
 * 【官方实现】使用贝叶斯推理更新单个栅格
 * 参考 range_sensor_layer.cpp:404-432
 */
void UltrasonicLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    // 计算栅格相对传感器的位置
    double dx = nx - ox;
    double dy = ny - oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx * dx + dy * dy);

    // 计算传感器模型概率
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }

    // 贝叶斯更新
    double prior = to_prob(getCost(x, y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ / (prob_occ + prob_not);

    // RCLCPP_INFO(
    //   logger_,
    //   "Cell update: dx=%.2f dy=%.2f theta=%.2f phi=%.2f sensor=%.2f prior=%.2f new=%.2f",
    //   dx, dy, theta, phi, sensor, prior, new_prob);

    unsigned char c = to_cost(new_prob);
    setCost(x, y, c);
  }
}

// ========== Layer 接口函数 ==========

/**
 * 【官方实现】更新边界
 * 参考 range_sensor_layer.cpp:440-477
 */
void UltrasonicLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // 滚动地图处理
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // 处理缓冲的消息，传递机器人位姿
  updateCostmap(robot_x, robot_y, robot_yaw);

  // 扩展边界
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();

  // 禁用检查
  if (!enabled_) {
    current_ = true;
    return;
  }

  // 超时检查
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

/**
 * 【官方实现】更新主代价地图
 * 参考 range_sensor_layer.cpp:479-524
 */
void UltrasonicLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_);
  unsigned char mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;

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

      if (old_cost == NO_INFORMATION || old_cost < current) {
        master_array[it] = current;
      }
      it++;
    }
  }

  buffered_readings_ = 0;

  // 恢复当前状态标志
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
}

/**
 * 【官方实现】重置层
 */
void UltrasonicLayer::reset()
{
  RCLCPP_DEBUG(logger_, "Resetting ultrasonic layer...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

/**
 * 【官方实现】停用层
 */
void UltrasonicLayer::deactivate()
{
  ultrasonic_msgs_buffer_.clear();
}

/**
 * 【官方实现】激活层
 */
void UltrasonicLayer::activate()
{
  ultrasonic_msgs_buffer_.clear();
}

// ========== 辅助函数 ==========

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
