/*
 * Ultrasonic Costmap Layer Plugin
 * 超声波代价地图层插件
 *
 * 功能：订阅超声波传感器数据，在代价地图中生成弧形/扇形障碍物
 *
 * 主要特性：
 * - 支持三个超声波传感器（左、中、右）
 * - 基于概率模型的传感器融合
 * - 支持射线清除（清除传感器到障碍物之间的自由空间）
 * - 支持传感器位置偏移配置
 */

 #ifndef ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
 #define ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_

 // 标准库头文件
 #include <memory>
 #include <string>
 #include <vector>
 #include <mutex>      // 线程安全
 #include <list>       // 消息缓冲队列
 #include <cmath>      // 数学函数
 #include <limits>     // 数值限制

 // ROS2 和 Nav2 头文件
 #include "rclcpp/rclcpp.hpp"
 #include "nav2_costmap_2d/costmap_layer.hpp"        // Costmap 层基类
 #include "nav2_costmap_2d/layered_costmap.hpp"      // 分层 Costmap
 #include "car_chassis/msg/ultrasonic.hpp"           // 超声波消息类型
 #include <tf2/time.h>  // TF2 时间类型
 
 namespace ultrasonic_costmap_layer
 {

 /**
  * @class UltrasonicLayer
  * @brief 超声波传感器代价地图层
  *
  * 此类从超声波传感器读取距离数据，并在代价地图中生成弧形/扇形障碍物。
  * 使用概率传感器模型来表示测量的不确定性。
  *
  * 继承自 nav2_costmap_2d::CostmapLayer，实现 Nav2 costmap 插件接口。
  */
 class UltrasonicLayer : public nav2_costmap_2d::CostmapLayer
 {
 public:
   /**
    * @brief 构造函数
    */
   UltrasonicLayer();

   /**
    * @brief 析构函数
    */
   virtual ~UltrasonicLayer();

   /**
    * @brief 初始化插件
    *
    * 由 Nav2 在加载插件时调用。执行以下操作：
    * - 加载所有 ROS2 参数
    * - 创建超声波话题订阅器
    * - 初始化内部状态
    */
   void onInitialize() override;

   /**
    * @brief 更新需要修改的代价地图区域边界
    *
    * 此函数在每次 costmap 更新周期被调用。它：
    * 1. 处理缓冲的超声波消息
    * 2. 更新内部 costmap
    * 3. 计算并返回被修改区域的边界框
    *
    * @param robot_x 机器人在全局坐标系中的 x 位置
    * @param robot_y 机器人在全局坐标系中的 y 位置
    * @param robot_yaw 机器人在全局坐标系中的航向角（弧度）
    * @param min_x 输出：更新区域的最小 x 坐标
    * @param min_y 输出：更新区域的最小 y 坐标
    * @param max_x 输出：更新区域的最大 x 坐标
    * @param max_y 输出：更新区域的最大 y 坐标
    */
   void updateBounds(
     double robot_x, double robot_y, double robot_yaw,
     double * min_x, double * min_y, double * max_x, double * max_y) override;

   /**
    * @brief 将本层的代价值合并到主代价地图中
    *
    * 使用概率阈值将本层的概率值转换为 costmap 代价值：
    * - 概率 > mark_threshold → LETHAL_OBSTACLE (254)
    * - 概率 < clear_threshold → FREE_SPACE (0)
    * - 中间值 → 保持不变
    *
    * 合并策略：
    * - 清除：仅在主地图为 NO_INFORMATION 或 FREE_SPACE 时
    * - 标记：仅在新代价高于现有代价时
    *
    * @param master_grid 主代价地图
    * @param min_i 更新窗口的最小 x 索引
    * @param min_j 更新窗口的最小 y 索引
    * @param max_i 更新窗口的最大 x 索引
    * @param max_j 更新窗口的最大 y 索引
    */
   void updateCosts(
     nav2_costmap_2d::Costmap2D & master_grid,
     int min_i, int min_j, int max_i, int max_j) override;

   /**
    * @brief 重置层状态
    *
    * 清除所有缓冲的消息和内部 costmap 数据
    */
   void reset() override;

   /**
    * @brief 停用层
    *
    * 清除消息缓冲区
    */
   void deactivate() override;

   /**
    * @brief 激活层
    *
    * 清除消息缓冲区（确保干净的启动）
    */
   void activate() override;

   /**
    * @brief 指示此层是否可被清除
    *
    * @return true 表示此层可以被清除命令清空
    */
   bool isClearable() override { return true; }

 private:
   // ========== 回调函数 ==========

   /**
    * @brief 超声波消息订阅回调
    *
    * 接收超声波数据并添加到缓冲区（线程安全）
    *
    * @param msg 超声波消息（包含 left, mid, right 三个距离值）
    */
   void bufferIncomingUltrasonicMsg(
     const car_chassis::msg::Ultrasonic::SharedPtr msg);

   // ========== 内部处理函数 ==========

   /**
    * @brief 处理缓冲的超声波消息，更新内部 costmap
    *
    * 批量处理所有缓冲的消息，对每条消息的三个传感器分别处理
    *
    * @param robot_x 机器人全局 x 位置
    * @param robot_y 机器人全局 y 位置
    * @param robot_yaw 机器人全局航向角
    */
   void updateCostmap(double robot_x, double robot_y, double robot_yaw);

   /**
    * @brief 处理单个传感器的读数
    *
    * 执行以下步骤：
    * 1. 验证读数有效性（范围检查）
    * 2. 应用距离缩放
    * 3. 确定传感器的位置偏移
    * 4. 将传感器位置从 base_link 转换到全局坐标系
    * 5. 调用 updateCostmapWithSensor 进行实际更新
    *
    * @param distance_mm 传感器读数（毫米）
    * @param sensor_angle 传感器安装角度（弧度，相对于 base_link）
    * @param stamp 时间戳
    * @param robot_x 机器人全局 x 位置
    * @param robot_y 机器人全局 y 位置
    * @param robot_yaw 机器人全局航向角
    */
   void processSingleSensor(
     int distance_mm, double sensor_angle, const rclcpp::Time & stamp,
     double robot_x, double robot_y, double robot_yaw);

   /**
    * @brief 使用传感器模型更新 costmap
    *
    * 这是核心算法函数，执行：
    * 1. 射线清除（可选）：清除传感器到障碍物之间的自由空间
    * 2. 计算扇形区域边界（三角形）
    * 3. 遍历边界框内的所有单元格
    * 4. 使用 orient2d 判断单元格是否在扇形内
    * 5. 对扇形内的单元格应用概率传感器模型
    *
    * @param distance 障碍物距离（米）
    * @param sensor_angle 传感器角度（弧度）
    * @param timestamp 时间戳（未使用）
    * @param clear_sensor_cone 是否清除整个扇形（当前未使用）
    * @param robot_x 传感器在全局坐标系的 x 位置
    * @param robot_y 传感器在全局坐标系的 y 位置
    * @param robot_yaw 机器人航向角
    * @param force_ray_clear_to_max 是否强制清除到最大量程
    */
   void updateCostmapWithSensor(
     double distance, double sensor_angle, const rclcpp::Time & stamp,
     bool clear_sensor_cone,
     double robot_x, double robot_y, double robot_yaw,
     bool force_ray_clear_to_max);

   /**
    * @brief 使用贝叶斯更新单个 costmap 单元格
    *
    * 应用概率传感器模型：
    * 1. 计算单元格相对于传感器的极坐标 (φ, θ)
    * 2. 使用 sensor_model() 计算传感器测量概率
    * 3. 使用贝叶斯公式结合先验概率
    * 4. 更新单元格代价值
    *
    * @param ox 传感器原点 x（全局坐标）
    * @param oy 传感器原点 y（全局坐标）
    * @param ot 传感器朝向（全局坐标系）
    * @param r 障碍物距离
    * @param nx 单元格中心 x（全局坐标）
    * @param ny 单元格中心 y（全局坐标）
    * @param clear 是否强制清除此单元格
    */
   void update_cell(
     double ox, double oy, double ot, double r,
     double nx, double ny, bool clear);

   // ========== 传感器模型函数 ==========

   /**
    * @brief 角度衰减函数 γ(θ)
    *
    * 计算角度对检测概率的影响。在 FOV 中心，权重为 1；
    * 在 FOV 边缘，权重降为 0。使用二次函数平滑过渡。
    *
    * @param theta 相对于传感器中心轴的角度（弧度）
    * @return 角度权重 [0, 1]
    */
   double gamma(double theta);

   /**
    * @brief 距离衰减函数 δ(φ)
    *
    * 使用 tanh 函数在障碍物距离附近创建平滑过渡。
    * phi_v 参数控制过渡的锐度。
    *
    * @param phi 单元格到传感器的径向距离
    * @return 距离权重 [0, 1]
    */
   double delta(double phi);

   /**
    * @brief 计算射线追踪的步长
    *
    * 根据给定角度计算在 x 和 y 方向上的步长，
    * 用于 Bresenham 风格的射线追踪。
    *
    * @param angle 射线角度
    * @param dx 输出：x 方向步长
    * @param dy 输出：y 方向步长
    */
   void get_deltas(double angle, double * dx, double * dy);

   /**
    * @brief 概率传感器模型
    *
    * 计算给定位置被占据的概率，基于：
    * - r: 传感器测量的障碍物距离
    * - φ (phi): 单元格到传感器的距离
    * - θ (theta): 单元格相对于传感器中心轴的角度
    *
    * 模型分段：
    * - φ < r - 2Δr: 远离障碍物，低概率
    * - r - 2Δr ≤ φ < r - Δr: 过渡区域
    * - r - Δr ≤ φ < r + Δr: 障碍物区域，高概率
    * - φ ≥ r + Δr: 障碍物后方，中等概率
    *
    * @param r 障碍物距离
    * @param phi 单元格距离
    * @param theta 单元格角度
    * @return 占据概率 [0, 1]
    */
   double sensor_model(double r, double phi, double theta);

   /**
    * @brief 检查距离读数是否有效
    *
    * @param distance 距离值（米）
    * @return true 如果在 [min_range, max_range] 内
    */
   bool isValidReading(double distance);

   /**
    * @brief 重置更新边界
    *
    * 将 min_x, min_y 设为最大值，max_x, max_y 设为最小值
    */
   void resetRange();

   // ========== ROS2 成员 ==========

   /// 超声波话题订阅器
   rclcpp::Subscription<car_chassis::msg::Ultrasonic>::SharedPtr ultrasonic_sub_;

   // ========== 线程安全与缓冲 ==========

   /// 保护消息缓冲区的互斥锁
   std::mutex ultrasonic_mutex_;

   /// 超声波消息缓冲队列（用于在回调和更新函数之间传递数据）
   std::list<car_chassis::msg::Ultrasonic> ultrasonic_msgs_buffer_;

   /// 已缓冲的读数计数
   size_t buffered_readings_{0};

   // ========== 层状态 ==========

   /// 数据是否是最新的
   bool current_{true};

   /// 是否刚被重置
   bool was_reset_{false};

   /// 默认单元格代价值（对应概率 0.5）
   unsigned char default_value_{0};

   // ========== 基础参数 ==========

   /// 是否启用此层
   bool enabled_{true};

   /// 超声波话题名称
   std::string ultrasonic_topic_;

   /// 全局坐标系名称
   std::string global_frame_;

   // ========== 传感器配置（角度单位：弧度）==========

   /// 左传感器安装角度（逆时针为正）
   double sensor_angle_left_{0.0};

   /// 中传感器安装角度
   double sensor_angle_mid_{0.0};

   /// 右传感器安装角度
   double sensor_angle_right_{0.0};

   // ========== 量程限制（单位：米）==========

   /// 最小有效量程
   double min_range_{0.05};

   /// 最大有效量程
   double max_range_{4.0};

   /// 距离缩放因子（真实距离 = 传感器值 × distance_scale）
   double distance_scale_{1.0};

   /// 单个传感器的视场角（弧度）
   double sensor_fov_{30.0 * M_PI / 180.0};

   // ========== 概率模型参数 ==========

   /// phi_v: 距离衰减函数的拐点，控制障碍物区域的锐度
   double phi_v_{1.2};

   /// 扇形区域膨胀系数（>1 扩大，<1 缩小）
   double inflate_cone_{1.0};

   /// 清除阈值：概率低于此值标记为自由空间
   double clear_threshold_{0.2};

   /// 标记阈值：概率高于此值标记为障碍物
   double mark_threshold_{0.8};

   /// 最大量程读数时是否清除整个射线
   bool clear_on_max_reading_{false};

   // ========== 射线清除参数 ==========

   /// 是否启用射线清除功能
   bool enable_ray_clear_{true};

   /// 清除到障碍物前的安全边距（米）
   double ray_clear_margin_{0.10};

   /// 射线清除半径（单元格数，0 = 单像素射线）
   int ray_clear_radius_cells_{0};

   // ========== 更新边界 ==========

   /// 本次更新的最小 x 坐标
   double min_x_{0.0};

   /// 本次更新的最小 y 坐标
   double min_y_{0.0};

   /// 本次更新的最大 x 坐标
   double max_x_{0.0};

   /// 本次更新的最大 y 坐标
   double max_y_{0.0};

   // ========== 超时配置 ==========

   /// 传感器数据超时时间（秒）
   double no_readings_timeout_{2.0};

   /// 最后一次接收到读数的时间
   rclcpp::Time last_reading_time_{0};

   /// TF 变换容差
   tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};

   // ========== 缓存变量 ==========

   /// 半视场角（sensor_fov / 2）
   double max_angle_{0.0};

   // ========== 传感器位置偏移（在 base_link 坐标系，单位：米）==========

   /// 左传感器 x 轴偏移
   double sensor_left_tx_{0.0};

   /// 左传感器 y 轴偏移
   double sensor_left_ty_{0.0};

   /// 中传感器 x 轴偏移
   double sensor_mid_tx_{0.0};

   /// 中传感器 y 轴偏移
   double sensor_mid_ty_{0.0};

   /// 右传感器 x 轴偏移
   double sensor_right_tx_{0.0};

   /// 右传感器 y 轴偏移
   double sensor_right_ty_{0.0};

   // ========== 拖影消除（Annulus Clear）==========

   /// 左传感器上次测量距离（米），用于环带清除
   double last_range_left_{-1.0};

   /// 中传感器上次测量距离（米），用于环带清除
   double last_range_mid_{-1.0};

   /// 右传感器上次测量距离（米），用于环带清除
   double last_range_right_{-1.0};

   // ========== 范围限制（避免远处"幽灵障碍物"）==========

   /// 是否启用范围外清除功能
   bool enable_out_of_range_clearing_{true};

   /// 有效范围半径（米），超出此范围的障碍物将被清除
   /// 默认为 -1.0，表示使用 costmap 大小的一半
   double clearing_range_{-1.0};

   /**
    * @brief 清除超出有效范围的障碍物
    *
    * 此函数清除远离机器人的旧障碍物，避免超声波在 local_costmap
    * 范围外留下"幽灵障碍物"。
    *
    * 策略：
    * 1. 计算机器人当前位置
    * 2. 遍历整个内部 costmap
    * 3. 将距离机器人超过 clearing_range 的单元格重置为默认值
    *
    * @param robot_x 机器人在全局坐标系的 x 位置
    * @param robot_y 机器人在全局坐标系的 y 位置
    */
   void clearOutOfRangeCells(double robot_x, double robot_y);

   /**
    * @brief 环带清除：清除从当前距离到上次距离之间的区域
    *
    * 当障碍物从远处移近（或消失）时，清除 [current_range, last_range]
    * 之间的环带区域，避免留下"拖影"。
    *
    * @param sensor_ox 传感器原点 x（全局坐标）
    * @param sensor_oy 传感器原点 y（全局坐标）
    * @param sensor_angle 传感器角度（全局坐标系）
    * @param current_range 当前测量距离
    * @param last_range 上次测量距离
    */
   void clearAnnulus(
     double sensor_ox, double sensor_oy, double sensor_angle,
     double current_range, double last_range);
 };

 }  // namespace ultrasonic_costmap_layer

 #endif  // ULTRASONIC_COSTMAP_LAYER__ULTRASONIC_LAYER_HPP_
 