/*代码功能：找到图片的红色区域 用四边形包含 并找到最大的四边形的中心点 绘制图像中心到四边形中心的连线 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    /* 从参数服务器读取 topic 名，launch 里可用 <param> 覆盖 */
    std::string in_topic  = nh_.param<std::string>("input_topic",  "camera/image_raw");
    std::string out_topic = nh_.param<std::string>("output_topic", "output_image_topic");

    image_sub_ = it_.subscribe(in_topic,  1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(out_topic, 1);
    ROS_INFO("ImageConverter initialized.");
    ROS_INFO("Subscribing to: %s", in_topic.c_str());
    ROS_INFO("Publishing to : %s", out_topic.c_str());
  }
  //只显示红色区域的部分
  void findRedColor(const cv::Mat& input_image, cv::Mat& output_image)
  {
    // 转换到HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

    // 定义红色的HSV范围
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(179, 255, 255);

    // 创建掩码
    cv::Mat mask1, mask2;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
    cv::Mat red_mask = mask1 | mask2;

    // 使用掩码提取红色区域
    output_image = cv::Mat::zeros(input_image.size(), input_image.type());
    input_image.copyTo(output_image, red_mask);
  }
  //用绿色四边形框住最大的红色区域 输出绿色四边形的中心点坐标
  void drawGreenSquare(cv::Mat& image)
  {
    cv::Mat red_image;
    findRedColor(image, red_image);

    cv::Mat hsv_image;
    cv::cvtColor(red_image, hsv_image, cv::COLOR_BGR2HSV);

    // 定义红色的HSV范围
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(20, 255, 255);
    cv::Scalar lower_red2(150, 100, 100);
    cv::Scalar upper_red2(179, 255, 255);

    // 创建掩码
    cv::Mat mask1, mask2;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
    cv::Mat red_mask = mask1 | mask2;

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
      cv::circle(image, image_center, 5, cv::Scalar(255, 0, 0), -1);

      // 获取最大轮廓的边界框
      if (found_largest)
      {
          cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);

          // 绘制绿色四边形
          cv::rectangle(image, bounding_box, cv::Scalar(0, 255, 0), 2);
          // 计算并输出中心点坐标
          cv::Point center(bounding_box.x + bounding_box.width / 2,
                          bounding_box.y + bounding_box.height / 2);
          //绘制四边形中心点
          cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);
          
          //绘制摄像机中心点到四边形中心点的连线
          cv::line(image, image_center, center, cv::Scalar(255, 0, 0), 1);
          ROS_INFO("Center of the largest red area: (%d, %d)", center.x, center.y);
      }else{
        ROS_INFO("No significant red area found.");
      }
    }
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

    // 使用OpenCV处理图像
    drawGreenSquare(cv_ptr->image);

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