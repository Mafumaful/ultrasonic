/*
 * Copyright (c) 2024
 * Based on Nav2 RangeSensorLayer
 *
 * Ultrasonic Costmap Layer - Header File
 * 基于官方 RangeSensorLayer 结构，适配三传感器超声波消息
 */

#ifndef ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
#define ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_

#include <list>
#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"  // 添加 CostmapLayer
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "car_chassis/msg/ultrasonic.hpp"  // 你的自定义消息类型

namespace ultrasonic_costmap_layer
{

/**
 * @class UltrasonicLayer
 * @brief 基于官方 RangeSensorLayer 的超声波传感器层
 *
 * 核心特性（保持官方实现）：
 * - 使用贝叶斯推理更新代价地图
 * - 概率传感器模型（gamma、delta 函数）
 * - 完整的 TF 坐标变换支持
 * - 锥形视场建模
 *
 * 自定义特性：
 * - 支持三传感器超声波消息（left, mid, right）
 * - 可配置的传感器安装角度
 * - 毫米到米的单位转换
 */
class UltrasonicLayer : public nav2_costmap_2d::CostmapLayer  // 改为 CostmapLayer
{
public:
  UltrasonicLayer();
  virtual ~UltrasonicLayer();

  /**
   * @brief 初始化层（Layer 接口）
   */
  virtual void onInitialize() override;

  /**
   * @brief 更新边界（Layer 接口）
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief 更新主代价地图（Layer 接口）
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief 重置层（Layer 接口）
   */
  virtual void reset() override;

  /**
   * @brief 激活层（Layer 接口）
   */
  virtual void activate() override;

  /**
   * @brief 停用层（Layer 接口）
   */
  virtual void deactivate() override;

  /**
   * @brief 是否可清除（Layer接口必须实现）
   */
  virtual bool isClearable() override { return true; }

private:
  /**
   * @brief 超声波消息回调
   */
  void ultrasonicCallback(const car_chassis::msg::Ultrasonic::SharedPtr msg);

  /**
   * @brief 缓冲传入的超声波消息（线程安全）
   */
  void bufferIncomingUltrasonicMsg(const car_chassis::msg::Ultrasonic::SharedPtr msg);

  /**
   * @brief 处理缓冲区中的所有消息
   */
  void updateCostmap();

  /**
   * @brief 处理单个传感器的数据
   * @param distance_mm 距离（毫米）
   * @param sensor_angle 传感器相对机器人的角度（弧度）
   * @param timestamp 消息时间戳
   */
  void processSingleSensor(
    int distance_mm,
    double sensor_angle,
    const rclcpp::Time & timestamp);

  /**
   * @brief 更新代价地图（带传感器数据）
   * @param distance 距离（米）
   * @param sensor_angle 传感器角度（弧度）
   * @param timestamp 时间戳
   * @param clear_sensor_cone 是否清除传感器锥形区域
   */
  void updateCostmapWithSensor(
    double distance,
    double sensor_angle,
    const rclcpp::Time & timestamp,
    bool clear_sensor_cone);

  /**
   * @brief 【官方函数】角度权重函数
   * gamma(θ) = 1 - (θ / max_angle)^2
   */
  double gamma(double theta);

  /**
   * @brief 【官方函数】距离衰减函数
   * delta(φ) = 1 - (1 + tanh(2*(φ - phi_v))) / 2
   */
  double delta(double phi);

  /**
   * @brief 【官方函数】传感器概率模型
   * 计算给定位置的障碍物存在概率
   */
  double sensor_model(double r, double phi, double theta);

  /**
   * @brief 【官方函数】更新单个栅格
   * 使用贝叶斯推理更新栅格概率
   */
  void update_cell(
    double ox, double oy, double ot, double r,
    double nx, double ny, bool clear);

  /**
   * @brief 【官方函数】计算扫描线步长
   */
  void get_deltas(double angle, double * dx, double * dy);

  /**
   * @brief 重置更新范围边界
   */
  void resetRange();

  /**
   * @brief 验证距离读数是否有效
   */
  bool isValidReading(double distance);

  // ========== 订阅者 ==========
  rclcpp::Subscription<car_chassis::msg::Ultrasonic>::SharedPtr ultrasonic_sub_;

  // ========== 数据缓冲（官方模式：线程安全缓冲）==========
  std::list<car_chassis::msg::Ultrasonic> ultrasonic_msgs_buffer_;
  std::mutex ultrasonic_mutex_;

  // ========== 状态变量 ==========
  bool current_;              // 数据是否最新
  bool was_reset_;            // 是否已重置
  int buffered_readings_;     // 缓冲读数计数
  rclcpp::Time last_reading_time_;  // 最后读数时间

  // ========== 参数（你的配置）==========
  std::string ultrasonic_topic_;     // 超声波话题名
  double sensor_angle_left_;         // 左传感器角度（弧度）
  double sensor_angle_mid_;          // 中间传感器角度（弧度）
  double sensor_angle_right_;        // 右传感器角度（弧度）
  double min_range_;                 // 最小有效距离（米）
  double max_range_;                 // 最大有效距离（米）
  double distance_scale_;            // 距离校准系数
  double sensor_fov_;                // 传感器视场角（弧度）
  double no_readings_timeout_;       // 无读数超时（秒）

  // ========== 参数（官方算法）==========
  double phi_v_;                     // 传感器模型参数（锐度）
  double inflate_cone_;              // 锥形膨胀系数 [0, 1]
  double clear_threshold_;           // 清除阈值
  double mark_threshold_;            // 标记阈值
  bool clear_on_max_reading_;        // 达到最大距离时是否清除

  // ========== 参数（清除优化）==========
  double clear_prob_before_;         // 测量距离之前的概率（加快清除）
  double clear_prob_beyond_;         // 超过测量距离的概率（清除远处残影）

  // ========== TF 和坐标系 ==========
  tf2::Duration transform_tolerance_;
  std::string global_frame_;

  // ========== 内部状态 ==========
  double max_angle_;                 // 当前传感器半视场角
  double min_x_, min_y_, max_x_, max_y_;  // 更新边界
};

}  // namespace ultrasonic_costmap_layer

#endif  // ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
