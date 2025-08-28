/*代码功能：将彩色图像变成灰度图 */
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

    // 使用OpenCV处理图像转换为灰度图
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // 将处理后的图像转换为ROS消息并发布
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // 保持相同的时间戳和帧ID
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image    = gray_image;

    image_pub_.publish(out_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_gray");
  ImageConverter ic;
  ros::spin();
  return 0;
}