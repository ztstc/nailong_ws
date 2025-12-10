#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <cmath>

struct Pose2D
{
    double x;
    double y;
    double theta;
};

class InertialPathNode
{
public:
    InertialPathNode()
        : nh_(),
          private_nh_("~"),
          mode_(IDLE),
          current_index_(0),
          have_odom_(false)
    {
        // 参数
        private_nh_.param("odom_topic", odom_topic_, std::string("odom"));
        private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("cmd_vel"));
        private_nh_.param("cmd_topic", cmd_topic_, std::string("inertial_path/cmd"));

        private_nh_.param("record_min_dist", record_min_dist_, 0.02);                 // 录制采点最小距离 2cm
        private_nh_.param("record_min_dtheta_deg", record_min_dtheta_deg_, 3.0);      // 录制采点最小转角 3度

        private_nh_.param("control_rate", control_rate_, 20.0);                       // 控制循环频率 Hz
        private_nh_.param("k_v", k_v_, 0.8);                                           // 线速度增益
        private_nh_.param("k_w", k_w_, 2.0);                                           // 角速度增益
        private_nh_.param("v_max", v_max_, 0.5);                                       // 最大线速度
        private_nh_.param("w_max", w_max_, 2.0);                                       // 最大角速度
        private_nh_.param("pos_tolerance", pos_tolerance_, 0.05);                      // 认为到达路径点的距离阈值 5cm
        private_nh_.param("angle_tolerance_deg", angle_tolerance_deg_, 10.0);         // 终点角度误差阈值（暂未使用）

        // 订阅和发布
        odom_sub_ = nh_.subscribe(odom_topic_, 50, &InertialPathNode::odomCallback, this);
        cmd_sub_  = nh_.subscribe(cmd_topic_, 10, &InertialPathNode::cmdCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

        // 控制定时器
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                         &InertialPathNode::controlTimerCallback, this);

        ROS_INFO("InertialPathNode initialized. odom: %s, cmd: %s, cmd_vel: %s",
                 odom_topic_.c_str(), cmd_topic_.c_str(), cmd_vel_topic_.c_str());
    }

private:
    enum Mode
    {
        IDLE,
        RECORD,
        PLAY_FORWARD,
        PLAY_BACKWARD
    };

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_pose_ = odomToPose2D(*msg);
        have_odom_ = true;

        if (mode_ == RECORD)
        {
            recordPose(current_pose_);
        }
    }

    void cmdCallback(const std_msgs::String::ConstPtr& msg)
    {
        const std::string& cmd = msg->data;
        if (cmd == "start_record")
        {
            startRecord();
        }
        else if (cmd == "stop_record")
        {
            stopRecord();
        }
        else if (cmd == "play_forward")
        {
            startPlayForward();
        }
        else if (cmd == "play_backward")
        {
            startPlayBackward();
        }
        else if (cmd == "stop")
        {
            stopAll();
        }
        else
        {
            ROS_WARN("Unknown inertial_path cmd: %s", cmd.c_str());
        }
    }

    void controlTimerCallback(const ros::TimerEvent&)
    {
        if (!have_odom_)
        {
            return;
        }

        if (mode_ == PLAY_FORWARD || mode_ == PLAY_BACKWARD)
        {
            followPathStep();
        }
    }

    // 模式控制
    void startRecord()
    {
        if (!have_odom_)
        {
            ROS_WARN("Cannot start record: no odom yet.");
            return;
        }
        path_.clear();
        path_.push_back(current_pose_);
        last_recorded_pose_ = current_pose_;
        mode_ = RECORD;
        ROS_INFO("Start recording path. First pose (%.3f, %.3f, %.1f deg)",
                 current_pose_.x, current_pose_.y, rad2deg(current_pose_.theta));
    }

    void stopRecord()
    {
        if (mode_ != RECORD)
        {
            ROS_WARN("stop_record: not in RECORD mode.");
            return;
        }
        mode_ = IDLE;
        ROS_INFO("Stop recording. Total %zu poses.", path_.size());
    }

    void startPlayForward()
    {
        if (path_.size() < 2)
        {
            ROS_WARN("Not enough path points to play forward.");
            return;
        }
        // 以当前姿态作为新的起点参考，计算整体偏移
        Pose2D ref_rec = path_.front();
        playback_offset_.dx = current_pose_.x - ref_rec.x;
        playback_offset_.dy = current_pose_.y - ref_rec.y;
        playback_offset_.dtheta = normalizeAngle(current_pose_.theta - ref_rec.theta);

        current_index_ = 0;
        mode_ = PLAY_FORWARD;
        ROS_INFO("Start play forward, %zu poses.", path_.size());
    }

    void startPlayBackward()
    {
        if (path_.size() < 2)
        {
            ROS_WARN("Not enough path points to play backward.");
            return;
        }
        // 以当前姿态作为新的终点参考（反向回放时对应录制终点）
        Pose2D ref_rec = path_.back();
        playback_offset_.dx = current_pose_.x - ref_rec.x;
        playback_offset_.dy = current_pose_.y - ref_rec.y;
        playback_offset_.dtheta = normalizeAngle(current_pose_.theta - ref_rec.theta);

        current_index_ = static_cast<int>(path_.size()) - 1;
        mode_ = PLAY_BACKWARD;
        ROS_INFO("Start play backward, %zu poses.", path_.size());
    }

    void stopAll()
    {
        mode_ = IDLE;
        publishZero();
        ROS_INFO("Stop inertial path actions.");
    }

    // 录制逻辑
    void recordPose(const Pose2D& pose)
    {
        double dx = pose.x - last_recorded_pose_.x;
        double dy = pose.y - last_recorded_pose_.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        double dtheta = std::fabs(normalizeAngle(pose.theta - last_recorded_pose_.theta));
        double dtheta_deg = rad2deg(dtheta);

        if (dist >= record_min_dist_ || dtheta_deg >= record_min_dtheta_deg_)
        {
            path_.push_back(pose);
            last_recorded_pose_ = pose;
            ROS_DEBUG("Record #%zu: (%.3f, %.3f, %.1f deg)",
                      path_.size(), pose.x, pose.y, rad2deg(pose.theta));
        }
    }

    // 回放控制
    void followPathStep()
    {
        if (path_.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Path empty in play mode.");
            mode_ = IDLE;
            publishZero();
            return;
        }

        if (current_index_ < 0 || current_index_ >= static_cast<int>(path_.size()))
        {
            ROS_INFO("Path finished.");
            mode_ = IDLE;
            publishZero();
            return;
        }

        Pose2D rec_target = path_[current_index_];

        // 将录制坐标系下的目标点整体平移/旋转到当前回放坐标系
        // 简化处理：只应用平移偏移，不强制匹配录制时的朝向，避免起点大幅扭头
        Pose2D target;
        target.x = rec_target.x + playback_offset_.dx;
        target.y = rec_target.y + playback_offset_.dy;
        target.theta = rec_target.theta; // 目前角度不做强制对齐，仅用于参考

        double dx = target.x - current_pose_.x;
        double dy = target.y - current_pose_.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < pos_tolerance_)
        {
            if (mode_ == PLAY_FORWARD)
                current_index_++;
            else if (mode_ == PLAY_BACKWARD)
                current_index_--;

            if (current_index_ < 0 || current_index_ >= static_cast<int>(path_.size()))
            {
                ROS_INFO("Reached end of path, stop.");
                mode_ = IDLE;
                publishZero();
            }
            return;
        }

        double yaw = current_pose_.theta;
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double x_r =  cos_yaw * dx + sin_yaw * dy;
        double y_r = -sin_yaw * dx + cos_yaw * dy;

        double v = k_v_ * x_r;
        double w = k_w_ * std::atan2(y_r, x_r);

        if (v >  v_max_) v =  v_max_;
        if (v < -v_max_) v = -v_max_;
        if (w >  w_max_) w =  w_max_;
        if (w < -w_max_) w = -w_max_;

        geometry_msgs::Twist cmd;
        cmd.linear.x  = v;
        cmd.angular.z = w;
        cmd_vel_pub_.publish(cmd);
    }

    // 工具函数
    static Pose2D odomToPose2D(const nav_msgs::Odometry& odom)
    {
        Pose2D p;
        p.x = odom.pose.pose.position.x;
        p.y = odom.pose.pose.position.y;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        p.theta = yaw;
        return p;
    }

    static double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double rad2deg(double r)
    {
        return r * 180.0 / M_PI;
    }

    void publishZero()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher  cmd_vel_pub_;
    ros::Timer      control_timer_;

    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string cmd_topic_;

    enum Mode mode_;
    int current_index_;
    bool have_odom_;

    Pose2D current_pose_;
    Pose2D last_recorded_pose_;
    std::vector<Pose2D> path_;

    struct PlaybackOffset
    {
        double dx{0.0};
        double dy{0.0};
        double dtheta{0.0};
    } playback_offset_;

    double record_min_dist_;
    double record_min_dtheta_deg_;
    double control_rate_;
    double k_v_;
    double k_w_;
    double v_max_;
    double w_max_;
    double pos_tolerance_;
    double angle_tolerance_deg_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inertial_path_node");
    InertialPathNode node;
    ros::spin();
    return 0;
}
