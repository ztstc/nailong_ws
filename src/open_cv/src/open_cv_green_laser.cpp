/*代码功能：
1.订阅激光雷达话题，解析激光雷达数据 并将距离数据叠加在摄像头画面上 当距离小于阈值时 用红色显示距离值 否则用绿色显示距离值
2.寻找画面中的绿色区域，并用绿色四边形框住最大的绿色区域，输出绿色四边形的中心点坐标 绘制图像中心到四边形中心的连线
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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber laser_sub_;
  std::vector<float> ranges; // 存储激光雷达距离数据
  float angle_min;           // 激光雷达最小角度（弧度）
  float angle_increment;     // 激光雷达角度增量（弧度）
  float range_min_ = 0.0f;   // 距离最小值（米）
  float range_max_ = 0.0f;   // 距离最大值（米）

  const double start_deg = 165.0; // 可读性：用度来表意
  const double end_deg   = 195.0;
public:
  ImageConverter()
    : it_(nh_)
  {
    /* 从参数服务器读取 topic 名，launch 里可用 <param> 覆盖 */
    std::string in_topic  = nh_.param<std::string>("input_topic",  "camera/image_raw");
    std::string out_topic = nh_.param<std::string>("output_topic", "output_image_topic");
    std::string laser_topic = nh_.param<std::string>("laser_topic", "scan");

    image_sub_ = it_.subscribe(in_topic,  1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(out_topic, 1);
    laser_sub_ = nh_.subscribe(laser_topic, 1, &ImageConverter::laserCb, this);
    ROS_INFO("ImageConverter initialized.");
    ROS_INFO("Image subscribing to: %s", in_topic.c_str());
    ROS_INFO("Laser subscribing to: %s", laser_topic.c_str());
    ROS_INFO("Image publishing to : %s", out_topic.c_str());
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

    if (!contours.empty())
    {
      // 找到最大的轮廓 并且过滤掉过小的轮廓
      size_t largest_contour_index = 0;
      bool found_largest = false;
      double largest_area = 0.0;
      for (size_t i = 0; i < contours.size(); ++i)
      {
          double area = cv::contourArea(contours[i]);
          if (area > largest_area && area > 2000) // 过滤掉小轮廓
          {
          largest_area = area;
          largest_contour_index = i;
          found_largest = true;
          }
      }
      //绘制摄像机中心点
      cv::Point image_center(image.cols / 2, image.rows / 2);
      cv::circle(image, image_center, 5, cv::Scalar(0, 0, 255), -1);

      // 获取最大轮廓的边界框
      if (found_largest)
      {
          cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);

          // 绘制红色四边形
          cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);
          // 计算并输出中心点坐标
          cv::Point center(bounding_box.x + bounding_box.width / 2,
                          bounding_box.y + bounding_box.height / 2);
          //绘制四边形中心点
          cv::circle(image, center, 5, cv::Scalar(255, 0, 0), -1);
          
          //绘制摄像机中心点到四边形中心点的连线
          cv::line(image, image_center, center, cv::Scalar(255, 0, 0), 1);
          ROS_INFO("Center of the largest green area: (%d, %d)", center.x, center.y);
      }else{
        ROS_INFO("No significant green area found.");
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

    // 正前方距离（0°）文本
    int idx0 = static_cast<int>(std::round((0.0 - angle_min) / angle_increment));
    idx0 = std::max(0, std::min(idx0, static_cast<int>(ranges.size()) - 1));
    float r0 = ranges[idx0];
    if (!std::isfinite(r0) || r0 < range_min_ || r0 > range_max_) {
      bool found = false;
      const int max_search = std::max(1, end_i - start_i);
      for (int d = 1; d <= max_search; ++d) {
        int i1 = idx0 - d, i2 = idx0 + d;
        if (i1 >= start_i) {
          float v = ranges[i1];
          if (std::isfinite(v) && v >= range_min_ && v <= range_max_) { r0 = v; found = true; break; }
        }
        if (i2 <= end_i) {
          float v = ranges[i2];
          if (std::isfinite(v) && v >= range_min_ && v <= range_max_) { r0 = v; found = true; break; }
        }
      }
      if (!found) r0 = std::numeric_limits<float>::quiet_NaN();
    }

    char buf[64];
    cv::Scalar txt_color(0, 255, 0);
    const double warn_thresh = 1.0; // 告警阈值（米）
    if (std::isfinite(r0)) {
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