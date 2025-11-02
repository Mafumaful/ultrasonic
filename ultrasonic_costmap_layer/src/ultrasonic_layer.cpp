/*
 * Ultrasonic Costmap Layer - Implementation
 * 超声波代价地图层 - 实现文件
 *
 * 此文件实现超声波传感器的概率模型和代价地图更新逻辑
 */

#include <angles/angles.h>        // 角度归一化
#include <algorithm>               // STL 算法
#include <list>                    // 消息缓冲队列
#include <limits>                  // 数值限制
#include <string>
#include <vector>
#include <cmath>                   // 数学函数

#include "pluginlib/class_list_macros.hpp"           // 插件导出宏
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "ultrasonic_costmap_layer/ultrasonic_layer.hpp"

// 导出插件类，使其可以被 pluginlib 动态加载
PLUGINLIB_EXPORT_CLASS(ultrasonic_costmap_layer::UltrasonicLayer, nav2_costmap_2d::Layer)

// 使用 Nav2 costmap 常量
using nav2_costmap_2d::LETHAL_OBSTACLE;  // 致命障碍物 (254)
using nav2_costmap_2d::NO_INFORMATION;    // 无信息 (255)
using nav2_costmap_2d::FREE_SPACE;        // 自由空间 (0)

namespace ultrasonic_costmap_layer
{

// ========== 辅助函数（来自 RangeSensorLayer）==========

/**
 * @brief 将概率值转换为代价值
 * 概率范围 [0, 1] 映射到代价值 [0, 252]
 * 注意：不使用 253-255，它们被保留为特殊值
 */
static inline unsigned char to_cost(double p) {
  return static_cast<unsigned char>(std::round(p * 252));
}

/**
 * @brief 将代价值转换为概率值
 */
static inline double to_prob(unsigned char c) {
  return static_cast<double>(c) / 252.0;
}

/**
 * @brief 计算三角形的有向面积（叉积）
 * 用于判断点是否在三角形内部
 * 返回值的符号表示点 c 相对于线段 ab 的方向
 */
static inline int orient2d(int ax, int ay, int bx, int by, int cx, int cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/**
 * @brief 计算三角形的面积
 */
static inline double area(int ax, int ay, int bx, int by, int cx, int cy)
{
  return std::fabs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0;
}

// ========== 构造函数 / 析构函数 ==========

/**
 * @brief 构造函数
 * 初始化成员变量（按照 hpp 中声明的顺序，避免编译器警告）
 */
UltrasonicLayer::UltrasonicLayer()
: buffered_readings_(0),
  current_(true),
  was_reset_(false),
  last_reading_time_(rclcpp::Time(0))
{
}

/**
 * @brief 析构函数
 * 使用默认实现（所有成员会自动析构）
 */
UltrasonicLayer::~UltrasonicLayer() = default;

// ========== 初始化 ==========

/**
 * @brief 插件初始化函数
 *
 * 由 Nav2 在加载插件时调用。执行以下操作：
 * 1. 初始化层状态
 * 2. 匹配父 costmap 的尺寸和分辨率
 * 3. 从参数服务器加载所有配置参数
 * 4. 创建超声波话题订阅器
 * 5. 打印配置信息到日志
 */
void UltrasonicLayer::onInitialize()
{
  // 初始化层状态
  current_ = true;                          // 数据是最新的
  was_reset_ = false;                       // 未被重置
  buffered_readings_ = 0;                   // 清空读数计数
  last_reading_time_ = clock_->now();       // 记录当前时间
  default_value_ = to_cost(0.5);            // 默认概率 0.5（未知状态）

  // 匹配父 costmap 的尺寸
  matchSize();
  // 将内部 costmap 的所有单元格设置为默认值（0.5 概率）
  setDefaultValue(default_value_);
  // 重置内部地图数据
  resetMaps();
  // 重置更新边界
  resetRange();

  // 获取 ROS2 节点指针（用于参数操作）
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ===== 加载基础参数 =====

  // 是否启用此层
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // 超声波话题名称
  declareParameter("ultrasonic_topic", rclcpp::ParameterValue("/ultrasonic"));
  node->get_parameter(name_ + "." + "ultrasonic_topic", ultrasonic_topic_);

  // ===== 加载传感器角度参数（度 → 弧度）=====

  declareParameter("sensor_angle_left", rclcpp::ParameterValue(45.0));
  declareParameter("sensor_angle_mid", rclcpp::ParameterValue(0.0));
  declareParameter("sensor_angle_right", rclcpp::ParameterValue(-45.0));

  double angle_left_deg{45.0}, angle_mid_deg{0.0}, angle_right_deg{-45.0};
  node->get_parameter(name_ + "." + "sensor_angle_left", angle_left_deg);
  node->get_parameter(name_ + "." + "sensor_angle_mid", angle_mid_deg);
  node->get_parameter(name_ + "." + "sensor_angle_right", angle_right_deg);

  // 转换为弧度存储
  sensor_angle_left_  = angle_left_deg  * M_PI / 180.0;
  sensor_angle_mid_   = angle_mid_deg   * M_PI / 180.0;
  sensor_angle_right_ = angle_right_deg * M_PI / 180.0;

  // ===== 加载量程参数（毫米 → 米）=====

  declareParameter("min_range", rclcpp::ParameterValue(50.0));
  declareParameter("max_range", rclcpp::ParameterValue(4000.0));
  double min_range_mm{50.0}, max_range_mm{4000.0};
  node->get_parameter(name_ + "." + "min_range", min_range_mm);
  node->get_parameter(name_ + "." + "max_range", max_range_mm);
  // 转换为米存储
  min_range_ = min_range_mm / 1000.0;
  max_range_ = max_range_mm / 1000.0;

  // 距离缩放因子
  declareParameter("distance_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "distance_scale", distance_scale_);

  // ===== 加载传感器视场角参数（度 → 弧度）=====

  declareParameter("sensor_fov", rclcpp::ParameterValue(30.0));
  double fov_deg{30.0};
  node->get_parameter(name_ + "." + "sensor_fov", fov_deg);
  sensor_fov_ = fov_deg * M_PI / 180.0;  // 转换为弧度

  // ===== 加载其他参数 =====

  // 传感器数据超时时间
  declareParameter("no_readings_timeout", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);

  // ===== 加载概率模型参数 =====

  // phi_v: 距离衰减函数的拐点，控制障碍物区域的锐度
  declareParameter("phi", rclcpp::ParameterValue(1.2));
  node->get_parameter(name_ + "." + "phi", phi_v_);

  // 扇形区域膨胀系数
  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);

  // 清除阈值：概率低于此值标记为自由空间
  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);

  // 标记阈值：概率高于此值标记为障碍物
  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);

  // 最大量程读数时是否清除整个射线
  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  // ===== 加载射线清除参数 =====

  // 是否启用射线清除功能
  declareParameter("enable_ray_clear", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enable_ray_clear", enable_ray_clear_);

  // 清除到障碍物前的安全边距（米）
  declareParameter("ray_clear_margin", rclcpp::ParameterValue(0.10));
  node->get_parameter(name_ + "." + "ray_clear_margin", ray_clear_margin_);

  // 射线清除半径（单元格数，0 = 单像素射线）
  declareParameter("ray_clear_radius_cells", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + "." + "ray_clear_radius_cells", ray_clear_radius_cells_);

  // ===== 加载传感器位置偏移参数（在 base_link 坐标系，单位：米）=====

  declareParameter("sensor_left_tx",  rclcpp::ParameterValue(0.0));
  declareParameter("sensor_left_ty",  rclcpp::ParameterValue(0.0));
  declareParameter("sensor_mid_tx",   rclcpp::ParameterValue(0.0));
  declareParameter("sensor_mid_ty",   rclcpp::ParameterValue(0.0));
  declareParameter("sensor_right_tx", rclcpp::ParameterValue(0.0));
  declareParameter("sensor_right_ty", rclcpp::ParameterValue(0.0));

  node->get_parameter(name_ + "." + "sensor_left_tx",  sensor_left_tx_);
  node->get_parameter(name_ + "." + "sensor_left_ty",  sensor_left_ty_);
  node->get_parameter(name_ + "." + "sensor_mid_tx",   sensor_mid_tx_);
  node->get_parameter(name_ + "." + "sensor_mid_ty",   sensor_mid_ty_);
  node->get_parameter(name_ + "." + "sensor_right_tx", sensor_right_tx_);
  node->get_parameter(name_ + "." + "sensor_right_ty", sensor_right_ty_);

  // ===== 加载 TF 变换容差 =====

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // 获取全局坐标系名称
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // ===== 创建超声波话题订阅器 =====

  ultrasonic_sub_ = node->create_subscription<car_chassis::msg::Ultrasonic>(
    ultrasonic_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&UltrasonicLayer::bufferIncomingUltrasonicMsg, this, std::placeholders::_1));

  // ===== 打印配置信息到日志 =====

  RCLCPP_INFO(logger_, "UltrasonicLayer: Initialized and subscribed to %s", ultrasonic_topic_.c_str());
  RCLCPP_INFO(logger_, "Sensor angles (deg): L=%.1f, M=%.1f, R=%.1f",
              angle_left_deg, angle_mid_deg, angle_right_deg);
  RCLCPP_INFO(logger_, "Range: %.3f-%.3f m, FOV: %.1f deg, phi: %.2f, ray_clear=%s, margin=%.2f m",
              min_range_, max_range_, fov_deg, phi_v_,
              enable_ray_clear_ ? "on" : "off", ray_clear_margin_);
  RCLCPP_INFO(logger_, "Sensor offsets (m): L(%.3f,%.3f) M(%.3f,%.3f) R(%.3f,%.3f)",
              sensor_left_tx_, sensor_left_ty_, sensor_mid_tx_, sensor_mid_ty_, sensor_right_tx_, sensor_right_ty_);
}

// ========== 传感器概率模型 ==========

/**
 * @brief 角度衰减函数 γ(θ)
 *
 * 计算角度对检测概率的影响。在 FOV 中心，权重为 1；
 * 在 FOV 边缘，权重降为 0。使用二次函数实现平滑过渡。
 *
 * 公式：γ(θ) = 1 - (θ / max_angle)²  当 |θ| ≤ max_angle
 *             = 0                     当 |θ| > max_angle
 *
 * @param theta 相对于传感器中心轴的角度（弧度）
 * @return 角度权重 [0, 1]
 */
double UltrasonicLayer::gamma(double theta)
{
  // 如果角度超出 FOV 范围，权重为 0
  if (std::fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    // 使用二次函数：离中心轴越远，权重越低
    return 1 - std::pow(theta / max_angle_, 2);
  }
}

/**
 * @brief 距离衰减函数 δ(φ)
 *
 * 使用 tanh 函数在障碍物距离附近创建平滑过渡。
 * phi_v 参数控制过渡的锐度（拐点位置）。
 *
 * 公式：δ(φ) = 1 - (1 + tanh(2(φ - phi_v))) / 2
 *
 * - 当 φ << phi_v 时，δ ≈ 1（远离障碍物，高权重）
 * - 当 φ ≈ phi_v 时，快速过渡
 * - 当 φ >> phi_v 时，δ ≈ 0（超过障碍物，低权重）
 *
 * @param phi 单元格到传感器的径向距离
 * @return 距离权重 [0, 1]
 */
double UltrasonicLayer::delta(double phi)
{
  return 1 - (1 + std::tanh(2 * (phi - phi_v_))) / 2;
}

/**
 * @brief 计算射线追踪的步长
 *
 * 根据给定角度计算在 x 和 y 方向上的步长，
 * 用于 Bresenham 风格的射线追踪。确保每步移动接近一个单元格大小。
 *
 * @param angle 射线角度（弧度）
 * @param dx 输出：x 方向步长（米）
 * @param dy 输出：y 方向步长（米）
 */
void UltrasonicLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = std::tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    // 根据 tan 值计算 x 步长
    *dx = resolution_ / ta;
  }
  // 根据角度的正负设置步长方向
  *dx = std::copysign(*dx, std::cos(angle));
  *dy = std::copysign(resolution_, std::sin(angle));
}

/**
 * @brief 概率传感器模型
 *
 * 计算给定位置被占据的概率。结合角度和距离的影响。
 *
 * 模型分为四个区域：
 * 1. φ < r - 2Δr: 远离障碍物区域，低概率
 * 2. r - 2Δr ≤ φ < r - Δr: 前过渡区，概率平滑上升
 * 3. r - Δr ≤ φ < r + Δr: 障碍物区域，高概率
 * 4. φ ≥ r + Δr: 障碍物后方，回到中等概率
 *
 * 其中 Δr = resolution_ 是单元格分辨率
 * λ = δ(φ) × γ(θ) 是综合权重因子
 *
 * @param r 传感器测量的障碍物距离
 * @param phi 单元格到传感器的距离
 * @param theta 单元格相对于传感器中心轴的角度
 * @return 占据概率 [0, 1]
 */
double UltrasonicLayer::sensor_model(double r, double phi, double theta)
{
  // 计算综合权重因子
  double lbda = delta(phi) * gamma(theta);
  double delta_val = resolution_;

  // 区域 1: 远离障碍物（传感器到障碍物之间）
  if (phi >= 0.0 && phi < r - 2 * delta_val * r) {
    return (1 - lbda) * (0.5);
  }
  // 区域 2: 接近障碍物的过渡区
  else if (phi < r - delta_val * r) {
    return lbda * 0.5 * std::pow((phi - (r - 2 * delta_val * r)) / (delta_val * r), 2) +
           (1 - lbda) * 0.5;
  }
  // 区域 3: 障碍物区域（最高概率）
  else if (phi < r + delta_val * r) {
    double J = (r - phi) / (delta_val * r);
    return lbda * ((1 - (0.5) * std::pow(J, 2)) - 0.5) + 0.5;
  }
  // 区域 4: 障碍物后方
  else {
    return 0.5;
  }
}

// ========== 消息处理 ==========

/**
 * @brief 超声波消息订阅回调（线程安全）
 *
 * 此函数在 ROS2 订阅器接收到消息时被调用。
 * 使用互斥锁保护消息缓冲区，确保线程安全。
 *
 * 前5条消息会被打印到日志，用于调试。
 *
 * @param msg 超声波消息（包含 left, mid, right 三个距离值，单位：毫米）
 */
void UltrasonicLayer::bufferIncomingUltrasonicMsg(
  const car_chassis::msg::Ultrasonic::SharedPtr msg)
{
  // 打印前5条消息用于调试
  static int print_cnt = 0;
  if (print_cnt < 5) {
    RCLCPP_INFO(logger_, "Ultrasonic rx: L=%d mm M=%d mm R=%d mm", msg->left, msg->mid, msg->right);
    ++print_cnt;
  }

  // 线程安全地添加消息到缓冲区
  std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
  ultrasonic_msgs_buffer_.push_back(*msg);
}


/**
 * @brief 批量处理缓冲的超声波消息
 *
 * 此函数在 updateBounds() 中被调用。它：
 * 1. 原子性地交换消息缓冲区（避免长时间持锁）
 * 2. 对每条消息的三个传感器（左、中、右）分别处理
 *
 * @param robot_x 机器人在全局坐标系的 x 位置
 * @param robot_y 机器人在全局坐标系的 y 位置
 * @param robot_yaw 机器人在全局坐标系的航向角（弧度）
 */
void UltrasonicLayer::updateCostmap(double robot_x, double robot_y, double robot_yaw)
{
  // 创建本地缓冲区副本（原子交换，最小化锁持有时间）
  std::list<car_chassis::msg::Ultrasonic> ultrasonic_buffer_copy;
  {
    std::lock_guard<std::mutex> lk(ultrasonic_mutex_);
    ultrasonic_msgs_buffer_.swap(ultrasonic_buffer_copy);
  }

  // 处理所有缓冲的消息
  for (auto & msg : ultrasonic_buffer_copy) {
    rclcpp::Time timestamp = clock_->now();  // 可以改用 msg.header.stamp

    // 分别处理三个传感器的读数
    processSingleSensor(msg.left,  sensor_angle_left_,  timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.mid,   sensor_angle_mid_,   timestamp, robot_x, robot_y, robot_yaw);
    processSingleSensor(msg.right, sensor_angle_right_, timestamp, robot_x, robot_y, robot_yaw);
  }
}

/**
 * @brief 处理单个传感器的读数
 *
 * 执行以下步骤：
 * 1. 将读数从毫米转换为米，并应用距离缩放
 * 2. 验证读数是否在有效范围内
 * 3. 根据传感器角度确定对应的位置偏移
 * 4. 将传感器位置从 base_link 坐标系转换到全局坐标系
 * 5. 调用 updateCostmapWithSensor() 进行实际更新
 *
 * @param distance_mm 传感器读数（毫米）
 * @param sensor_angle 传感器安装角度（弧度，相对于 base_link x 轴）
 * @param timestamp 时间戳
 * @param robot_x 机器人在全局坐标系的 x 位置
 * @param robot_y 机器人在全局坐标系的 y 位置
 * @param robot_yaw 机器人在全局坐标系的航向角（弧度）
 */
void UltrasonicLayer::processSingleSensor(
  int distance_mm,
  double sensor_angle,
  const rclcpp::Time & timestamp,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  // 1) 转换距离：毫米 → 米，并应用缩放因子
  double distance = distance_mm * distance_scale_ / 1000.0;

  // 2) 验证读数有效性
  if (!isValidReading(distance)) {
    return;  // 无效读数，直接返回
  }

  // 3) 确定是否需要特殊处理（最大量程读数）
  bool clear_sensor_cone = false;
  bool force_ray_clear_to_max = false;
  if (distance >= max_range_ - 1e-2 && clear_on_max_reading_) {
    // 如果读数接近最大量程，且启用了最大量程清除
    force_ray_clear_to_max = true;
  }

  // 4) 根据传感器角度选择对应的位置偏移
  // 这些偏移是在 base_link 坐标系中定义的
  double tx_b = 0.0, ty_b = 0.0;
  if (sensor_angle == sensor_angle_left_) {
    tx_b = sensor_left_tx_;
    ty_b = sensor_left_ty_;
  } else if (sensor_angle == sensor_angle_mid_) {
    tx_b = sensor_mid_tx_;
    ty_b = sensor_mid_ty_;
  } else if (sensor_angle == sensor_angle_right_) {
    tx_b = sensor_right_tx_;
    ty_b = sensor_right_ty_;
  }

  // 5) 将传感器位置从 base_link 转换到全局坐标系
  // 使用 2D 旋转矩阵：
  //   [cos(yaw)  -sin(yaw)] [tx_b]   [robot_x]
  //   [sin(yaw)   cos(yaw)] [ty_b] + [robot_y]
  const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
  const double sensor_ox = robot_x + c * tx_b - s * ty_b;  // 全局 x
  const double sensor_oy = robot_y + s * tx_b + c * ty_b;  // 全局 y

  // 6) 使用传感器的真实位置进行 costmap 更新
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
