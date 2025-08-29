/*代码功能：
1.订阅激光雷达话题，解析激光雷达数据 并将距离数据叠加在摄像头画面上 当距离小于阈值时 用红色显示距离值 否则用绿色显示距离值
2.寻找画面中的绿色区域，并用绿色四边形框住最大的绿色区域，输出绿色四边形的中心点坐标 绘制图像中心到四边形中心的连线
3.发布cmd_vel控制小车 朝向绿色区域前进
*/
//默认分辨率1280x720
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <limits>
#include <geometry_msgs/Twist.h>

class ImageConverter
{
  ros::NodeHandle nh_; // 私有 NodeHandle
  ros::NodeHandle pnh_{"~"}; // 新增：私有命名空间句柄，用于读取~参数
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber laser_sub_;
  ros::Publisher cmd_vel_pub_;    // 发布cmd_vel控制小车
  ros::Timer delta_timer_; // 定时器成员
  std::vector<float> ranges; // 存储激光雷达距离数据
  float angle_min;           // 激光雷达最小角度（弧度）
  float angle_increment;     // 激光雷达角度增量（弧度）
  float range_min_ = 0.0f;   // 距离最小值（米）
  float range_max_ = 0.0f;   // 距离最大值（米）

  const double start_deg = 165.0; // 可读性：用度来表意
  const double end_deg   = 195.0;

  float r0 = 0;

  int16_t delta = 0; //记录图像中心到绿色区域中心的水平偏移量

  // === 新增：PID/滤波/检测/运动 参数与状态 ===
  // PID参数与内部状态
  double kp_=0.004, ki_=0.0, kd_=0.0015, d_alpha_=0.2;
  double i_term_=0.0, prev_error_=0.0, d_term_filt_=0.0;
  bool pid_initialized_=false;

  // 误差与距离的低通滤波
  double delta_alpha_=0.2;             // 图像偏差低通系数
  double delta_filt_=0.0;              // 滤波后的偏差
  bool   delta_filt_init_=false;

  double r0_alpha_=0.3;                // 距离低通系数
  double r0_filt_=std::numeric_limits<double>::quiet_NaN();
  bool   r0_filt_init_=false;

  // 目标可见性与丢失处理
  bool target_visible_=false;
  int  lost_count_=0;
  int  lost_count_max_=10;

  // 绿色检测形态学与面积阈值
  int    erode_iter_=1, dilate_iter_=1;
  double area_min_=2000.0;

  // 线速度控制参数
  double v_max_=0.6, v_min_=0.1, stop_dist_=0.7, full_dist_=1.2;
  // === 新增End ===

public:
  ImageConverter()
    : nh_(), pnh_("~"), it_(nh_)
  {
    /* 从参数服务器读取 topic 名，launch 里可用 <param> 覆盖 */
    std::string in_topic  = pnh_.param<std::string>("input_topic",  "camera/image_raw");
    std::string out_topic = pnh_.param<std::string>("output_topic", "output_image_topic");
    std::string laser_topic = pnh_.param<std::string>("laser_topic", "scan");
    std::string cmd_vel_topic = pnh_.param<std::string>("cmd_vel_topic", "cmd_vel");

    image_sub_ = it_.subscribe(in_topic,  1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(out_topic, 1);
    laser_sub_ = nh_.subscribe(laser_topic, 1, &ImageConverter::laserCb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    ROS_INFO("ImageConverter initialized.");
    ROS_INFO("Image subscribing to: %s", in_topic.c_str());
    ROS_INFO("Laser subscribing to: %s", laser_topic.c_str());
    ROS_INFO("Image publishing to : %s", out_topic.c_str());
    ROS_INFO("Cmd_vel publishing to : %s", cmd_vel_topic.c_str());

    // === 从参数服务器读取PID/滤波/形态学/运动参数（使用~私有命名空间） ===
    pnh_.param("pid/kp", kp_, kp_);
    pnh_.param("pid/ki", ki_, ki_);
    pnh_.param("pid/kd", kd_, kd_);
    pnh_.param("pid/d_alpha", d_alpha_, d_alpha_);

    pnh_.param("filter/delta_alpha", delta_alpha_, delta_alpha_);
    pnh_.param("filter/r0_alpha", r0_alpha_, r0_alpha_);

    pnh_.param("detect/erode_iter", erode_iter_, erode_iter_);
    pnh_.param("detect/dilate_iter", dilate_iter_, dilate_iter_);
    pnh_.param("detect/area_min", area_min_, area_min_);

    pnh_.param("motion/v_max", v_max_, v_max_);
    pnh_.param("motion/v_min", v_min_, v_min_);
    pnh_.param("motion/stop_dist", stop_dist_, stop_dist_);
    pnh_.param("motion/full_dist", full_dist_, full_dist_);
    pnh_.param("motion/lost_count_max", lost_count_max_, lost_count_max_);
    // === 新增End ===

    // 初始化定时器，每 0.1 秒调用一次
    delta_timer_ = nh_.createTimer(ros::Duration(0.1), &ImageConverter::deltaTimerCb, this);
  }
  // 激光雷达回调函数
  void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    //更新距离数组
    ranges = msg->ranges;
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;
    range_min_ = msg->range_min;
    range_max_ = msg->range_max;
  }

  //只显示绿色区域的部分
  void findGreenColor(const cv::Mat& input_image, cv::Mat& output_image)
  {
    if (input_image.empty()) { output_image.release(); return; }

    // 转换到HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

    // 定义绿色的HSV范围
    cv::Scalar lower_green(40, 100, 100);
    cv::Scalar upper_green(80, 255, 255);

    // 创建掩码（修复：移除未初始化的 mask2，避免空矩阵按位或）
    cv::Mat mask;
    cv::inRange(hsv_image, lower_green, upper_green, mask);

    // === 新增：形态学滤波，去噪与连通增强 ===
    if (erode_iter_ > 0)  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), erode_iter_);
    if (dilate_iter_ > 0) cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), dilate_iter_);
    // === 新增End ===

    cv::Mat green_mask = mask;

    // 使用掩码提取绿色区域
    output_image = cv::Mat::zeros(input_image.size(), input_image.type());
    input_image.copyTo(output_image, green_mask);
  }

  //用红色四边形框住最大的绿色区域 输出绿色四边形的中心点坐标 
  void drawRedSquare(cv::Mat& image)
  {
    if (image.empty()) return;

    cv::Mat green_image;
    findGreenColor(image, green_image);
    if (green_image.empty()) return;

    cv::Mat hsv_image;
    cv::cvtColor(green_image, hsv_image, cv::COLOR_BGR2HSV);

    // 定义绿色的HSV范围
    cv::Scalar lower_green(40, 100, 100);
    cv::Scalar upper_green(80, 255, 255);

    // 创建掩码
    cv::Mat mask1;
    cv::inRange(hsv_image, lower_green, upper_green, mask1);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask1, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 预设为未可见
    bool found_largest = false;
    target_visible_ = false;

    if (!contours.empty())
    {
      // 找到最大的轮廓 并且过滤掉过小的轮廓（使用可调阈值）
      size_t largest_contour_index = 0;
      double largest_area = 0.0;
      for (size_t i = 0; i < contours.size(); ++i)
      {
        double area = cv::contourArea(contours[i]);
        if (area > largest_area && area > area_min_)
        {
          largest_area = area;
          largest_contour_index = i;
          found_largest = true;
        }
      }

      //绘制摄像机中心点
      cv::Point image_center(image.cols / 2, image.rows / 2);
      cv::circle(image, image_center, 5, cv::Scalar(0, 0, 255), -1);

      if (found_largest)
      {
        cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);
        // 绘制红色四边形
        cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);
        // 计算中心点并绘制
        cv::Point center(bounding_box.x + bounding_box.width / 2,
                         bounding_box.y + bounding_box.height / 2);
        cv::circle(image, center, 5, cv::Scalar(255, 0, 0), -1);
        cv::line(image, image_center, center, cv::Scalar(255, 0, 0), 1);
        ROS_INFO("Center of the largest green area: (%d, %d)", center.x, center.y);

        // === 新增：对delta进行低通滤波，稳定控制 ===
        double delta_raw = static_cast<double>(center.x - image_center.x);
        if (!delta_filt_init_) { delta_filt_ = delta_raw; delta_filt_init_ = true; }
        else { delta_filt_ = delta_alpha_ * delta_raw + (1.0 - delta_alpha_) * delta_filt_; }
        delta = static_cast<int16_t>(std::lround(delta_filt_));
        target_visible_ = true;
        lost_count_ = 0;
        // === 新增End ===
      }
      else
      {
        // 未找到明显绿色区域：保持上次滤波值，标记丢失
        ROS_INFO("No significant green area found.");
        target_visible_ = false;
        if (lost_count_ < 1000000) ++lost_count_;
      }
    }
  }

  //将激光雷达距离数据显示在图像上 只显示165度-195度范围内的距离
  void drawLaser(cv::Mat& image)
  {
    if (image.empty() || ranges.empty() || angle_increment <= 0.0f) return;

    // 配置
    const double window_deg = 30.0;                      // 总宽度 30°
    const double half_win_rad = (window_deg * M_PI / 180.0) / 2.0;
    const float  min_disp = std::max(0.05f, range_min_); // 最小显示距离
    const float  display_cap = (range_max_ > 0.0f ? std::min(range_max_, 8.0f) : 8.0f); // 显示上限，提升近距离起伏
    const int margin_h = 10;
    const int margin_bottom = 10;
    const int graph_height = std::max(60, std::min(180, image.rows / 3));
    const int base_y = image.rows - margin_bottom;
    const int left_x = margin_h;
    const int right_x = image.cols - margin_h;

    // 角度与索引范围（以 0 弧度为正前方）
    const double start_rad = -half_win_rad;
    const double end_rad   =  half_win_rad;
    int start_i = static_cast<int>(std::ceil((start_rad - angle_min) / angle_increment));
    int end_i   = static_cast<int>(std::floor((end_rad   - angle_min) / angle_increment));
    start_i = std::max(0, start_i);
    end_i   = std::min(end_i, static_cast<int>(ranges.size()) - 1);
    if (start_i >= end_i) return;

    // 计算窗口内有效距离的动态范围，并缓存裁剪后的距离
    float vmin = std::numeric_limits<float>::infinity();
    float vmax = 0.0f;
    std::vector<float> r_clamps(end_i - start_i + 1, display_cap);
    for (int i = start_i; i <= end_i; ++i) {
      float r = ranges[i];
      if (std::isfinite(r) && r >= range_min_ && r <= range_max_) {
        float rc = std::min(std::max(r, min_disp), display_cap);
        r_clamps[i - start_i] = rc;
        vmin = std::min(vmin, rc);
        vmax = std::max(vmax, rc);
      }
    }
    if (!std::isfinite(vmin)) { vmin = min_disp; vmax = display_cap; } // 全无效时回退
    if (vmax - vmin < 1e-3f) {                                         // 确保有可视跨度
      vmax = std::min(display_cap, vmin + 0.5f);
      vmin = std::max(min_disp,  vmax - 0.5f);
    }

    // 构建折线点（修正镜像：左=+角度，右=-角度）
    std::vector<cv::Point> pts;
    pts.reserve(end_i - start_i + 1);
    const double denom_ang = (end_rad - start_rad);
    for (int i = start_i; i <= end_i; ++i) {
      double ang = angle_min + i * angle_increment; // [-15°, +15°]
      double t_img = (end_rad - ang) / denom_ang;   // +角度在左侧
      int x = static_cast<int>(left_x + t_img * (right_x - left_x));

      float rc = r_clamps[i - start_i];
      int y = static_cast<int>(base_y - graph_height * ((vmax - rc) / (vmax - vmin)));
      pts.emplace_back(x, y);
    }

    // 基线与折线
    cv::line(image, cv::Point(left_x, base_y), cv::Point(right_x, base_y), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (pts.size() >= 2) {
      cv::polylines(image, pts, false, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    // 中心 0° 参考线
    double t0 = (end_rad - 0.0) / denom_ang; // 0°
    int x0 = static_cast<int>(left_x + t0 * (right_x - left_x));
    cv::line(image, cv::Point(x0, base_y - graph_height), cv::Point(x0, base_y), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    // 替换：正前方距离估计，使用±1°窗口内的有效测距均值
    {
      const double win_rad = 1.0 * M_PI / 180.0; // 1°
      int start_i2 = static_cast<int>(std::ceil(( -win_rad - angle_min) / angle_increment));
      int end_i2   = static_cast<int>(std::floor(( +win_rad - angle_min) / angle_increment));
      start_i2 = std::max(0, start_i2);
      end_i2   = std::min(end_i2, static_cast<int>(ranges.size()) - 1);

      float sum = 0.0f;
      int   cnt = 0;
      for (int i = start_i2; i <= end_i2; ++i) {
        float v = ranges[i];
        if (std::isfinite(v) && v >= range_min_ && v <= range_max_) {
          sum += v;
          ++cnt;
        }
      }
      if (cnt > 0) r0 = sum / static_cast<float>(cnt);
      else         r0 = std::numeric_limits<float>::quiet_NaN();
    }

    // 保持：对r0进行低通滤波，稳定线速度控制
    if (std::isfinite(r0)) {
      if (!r0_filt_init_) { r0_filt_ = r0; r0_filt_init_ = true; }
      else { r0_filt_ = r0_alpha_ * r0 + (1.0 - r0_alpha_) * r0_filt_; }
    }

    char buf[64];
    cv::Scalar txt_color(0, 255, 0);
    const double warn_thresh = 1.0; // 告警阈值（米）
    if (std::isfinite(r0) && r0 != 0) {
      std::snprintf(buf, sizeof(buf), "Distance: %.2f m", r0);
      if (r0 < warn_thresh) txt_color = cv::Scalar(0, 0, 255);
    } else {
      std::snprintf(buf, sizeof(buf), "Distance: Invalid");
      txt_color = cv::Scalar(0, 0, 255);
    }
    int baseline = 0;
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 2;
    cv::Size ts = cv::getTextSize(buf, font, font_scale, thickness, &baseline);
    cv::putText(image, buf, cv::Point(10, 10 + ts.height), font, font_scale, txt_color, thickness, cv::LINE_AA);
  }

  // === 改造：根据PID与滤波误差控制速度（接收dt） ===
  void delta2Speed(double dt)
  {
    // 使用滤波后的delta作为误差
    double error = delta_filt_;
    if (!pid_initialized_) {
      prev_error_ = error;
      d_term_filt_ = 0.0;
      pid_initialized_ = true;
    }

    // PID微分项引入一阶滤波抑制噪声
    double d_error = (error - prev_error_) / std::max(1e-3, dt);
    d_term_filt_ = d_alpha_ * d_error + (1.0 - d_alpha_) * d_term_filt_;
    // 积分项（带简单防饱和）
    i_term_ += error * dt;
    double i_limit = 10000.0;
    if (i_term_ > i_limit) i_term_ = i_limit;
    if (i_term_ < -i_limit) i_term_ = -i_limit;

    // 角速度（保持与原始符号一致：误差为右正，需右转 => 负号）
    double angular = -(kp_ * error + ki_ * i_term_ + kd_ * d_term_filt_);
    // 线速度：基于滤波后的r0
    double linear = 0.0;
    double r = r0_filt_;

    if (target_visible_) {
      if (std::isfinite(r)) {
        if (r <= stop_dist_) {
          linear = 0.0;
        } else if (r >= full_dist_) {
          linear = v_max_;
        } else {
          double t = (r - stop_dist_) / std::max(1e-3, (full_dist_ - stop_dist_));
          linear = v_min_ + t * (v_max_ - v_min_);
        }
      } else {
        // 无有效距离时，缓慢前进
        linear = v_min_;
      }
    } else {
      // 丢失目标：停止前进，并原地右转搜索
      linear = 0.0;
      angular = -0.6;
    }

    // 近距离安全刹停
    if (std::isfinite(r) && r < 0.4) {
      linear = 0.0;
    }

    // 限幅
    if (angular > 3.0) angular = 3.0;
    if (angular < -3.0) angular = -3.0;

    prev_error_ = error;
    cmdPublish(static_cast<float>(linear), static_cast<float>(angular));
  }

  void cmdPublish(float linear, float angular)
  {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = linear;   // 线速度
    cmd_msg.angular.z = angular; // 角速度
    cmd_vel_pub_.publish(cmd_msg);
  }

  // 定时器回调函数（提供dt给PID）
  void deltaTimerCb(const ros::TimerEvent& ev)
  {
    double dt = (ev.current_real - ev.last_real).toSec();
    if (!(dt > 0.0)) dt = 0.1;
    delta2Speed(dt);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (!cv_ptr || cv_ptr->image.empty()) {
      ROS_WARN("Empty image received, skip processing.");
      return;
    }

    // 使用OpenCV处理图像
    drawRedSquare(cv_ptr->image);
    drawLaser(cv_ptr->image);
    

    // 将处理后的图像转换为ROS消息并发布
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // 保持相同的时间戳和帧ID
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = cv_ptr ->image;

    image_pub_.publish(out_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_red");
  ImageConverter ic;
  ros::spin();
  return 0;
}