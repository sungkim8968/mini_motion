#include <gazebo/common/Events.hh>
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "../include/parabolic_motion.hpp"

#ifdef IGN_PROFILER_ENABLE
#include "ignition/common/Profiler.hh"
#endif
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosParabolicMotionPrivate
{
    public:
        // 每次仿真迭代时的回调函数
        void OnUpdate(const gazebo::common::UpdateInfo & _info);
        // 接收速度命令时的回调函数
        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
        // 更新里程计函数
        void UpdateOdometry(const gazebo::common::Time & _current_time);
        // 发布里程计函数
        void PublishOdometryTf(const gazebo::common::Time & _current_time);
        
        // 初始化 指向gazeboRos的节点
        gazebo_ros::Node::SharedPtr ros_node_;
        // 订阅速度指令
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        // 发布者里程计
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
        // 发布TF
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        // 接收到的速度指令
        geometry_msgs::msg::Twist target_cmd_vel_;
        // 保持最近的里程计信息
        nav_msgs::msg::Odometry odom_;
        // gazebo中指向world的指针
        gazebo::physics::WorldPtr world_;
        // gazebo中指向model的指针
        gazebo::physics::ModelPtr model_;
        //gazebo中world 迭代 连接事件
        gazebo::event::ConnectionPtr update_connection_;
        // 保护回调函数访问的变量
        std::mutex lock_;
        // 更新频率
        double update_period_;
        // 发布频率
        double publish_period_;
        // 上次更新时间
        gazebo::common::Time last_update_time_;
        // 上次发布时间
        gazebo::common::Time last_publish_time_;

        // 里程计的frame id
        std::string odometry_frame_;
        // 机器人的base frame id；
        std::string robot_base_frame_;
        // 是否发布里程计信息
        bool publish_odometry_;
        // 是否发布odom-to-world转换
        bool publish_odom_tf_;
};  

GazeboRosParabolicMotion::GazeboRosParabolicMotion()
: impl_(std::make_unique<GazeboRosParabolicMotionPrivate>())
{
}
GazeboRosParabolicMotion::~GazeboRosParabolicMotion()
{
}

void GazeboRosParabolicMotion::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // 将指针间的模型传递
    impl_->model_ = _model;
    impl_->world_ = _model->GetWorld();
    // 初始化ros节点
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
    // 获取Qos配置文件
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    // 里程计
    impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
    impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
    // 更新速率
    auto update_rate = _sdf->Get<double>("update_rate",20.0).first;
    if (update_rate >0.0)
    {
        impl_->update_period_ = 1.0/update_rate;
    } else{
        impl_->update_period_ = 0.0;
    }
    impl_->last_update_time_ = impl_->world_->SimTime();
    // 更新发布速率
    auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first;
    if (publish_rate >0.0)
    {
        impl_->publish_period_ = 1.0/publish_rate;
    } else {
        impl_->publish_period_ = 0.0;
    }
    impl_->last_publish_time_ = impl_->world_->SimTime();
    // 更新订阅速率
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1))
                            , std::bind(&GazeboRosParabolicMotionPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

    // 广播里程计topic
    impl_->publish_odometry_ = _sdf->Get<bool>("publish_odom", true).first;
    if (impl_->publish_odometry_)
    {
        impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]", impl_->odometry_pub_->get_topic_name());
    }
    // 广播TF
    impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
    if (impl_->publish_odom_tf_)
    {
        impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing odom trtansform between [%s] and [%s]", impl_->odometry_frame_.c_str(), impl_->robot_base_frame_.c_str());
    }
    // 获取并设置协方差
    auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
    auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
    auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

    impl_->odom_.pose.covariance[0] = covariance_x;
    impl_->odom_.pose.covariance[7] = covariance_y;
    impl_->odom_.pose.covariance[14] = 1000000000000.0;
    impl_->odom_.pose.covariance[21] = 1000000000000.0;
    impl_->odom_.pose.covariance[28] = 1000000000000.0;
    impl_->odom_.pose.covariance[35] = covariance_yaw;

    impl_->odom_.twist.covariance[0] = covariance_x;
    impl_->odom_.twist.covariance[7] = covariance_y;
    impl_->odom_.twist.covariance[14] = 1000000000000.0;
    impl_->odom_.twist.covariance[21] = 1000000000000.0;
    impl_->odom_.twist.covariance[28] = 1000000000000.0;
    impl_->odom_.twist.covariance[35] = covariance_yaw;
    
    // 设置odom的header
    impl_->odom_.header.frame_id = impl_->odometry_frame_;
    impl_->odom_.child_frame_id = impl_->robot_base_frame_;
    // 监听更新事件 每一次仿真迭代
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosParabolicMotionPrivate::OnUpdate, impl_.get(), std::placeholders::_1)
    );
}

// 重置函数
void GazeboRosParabolicMotion::Reset()
{
    impl_->last_update_time_ = impl_->world_->SimTime();
    impl_->target_cmd_vel_.linear.x = 0;
    impl_->target_cmd_vel_.linear.y = 0;
    impl_->target_cmd_vel_.angular.z = 0;
}

// 更新函数
void GazeboRosParabolicMotionPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    std::lock_guard<std::mutex> scoped_lock(lock_);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE("GazaboRosParabolicMotionPrivate::OnUpdate");
        IGN_PROFILE_BEGIN("fill ROS message");
    #endif
    if (seconds_since_last_update > update_period_)
    {
        ignition::math::Vector3d velocity(target_cmd_vel_.linear.x, target_cmd_vel_.linear.y, target_cmd_vel_.linear.z);
        model_->SetLinearVel(velocity);
        last_update_time_ = _info.simTime;
    }
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
        if (publish_odometry_ || publish_odom_tf_)
        {
            double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();
            if (seconds_since_last_publish < publish_period_)
            {
                return;
            }
            
        
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_BEGIN("UpdateOdometry");
    #endif
        UpdateOdometry(_info.simTime);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
        if (publish_odometry_)
        {
            #ifdef IGN_PROFILER_ENABLE
                IGN_PROFILE_BEGIN("publish odometry");
            #endif
                odometry_pub_->publish(odom_);
            #ifdef IGN_PROFILER_ENABLE
                IGN_PROFILE_END();
            #endif
        }
        if (publish_odom_tf_) {
            #ifdef IGN_PROFILER_ENABLE
                IGN_PROFILE_BEGIN("publish odometryTF");
            #endif
                PublishOdometryTf(_info.simTime);
            #ifdef IGN_PROFILER_ENABLE
                IGN_PROFILE_END();
            #endif
        }
        last_publish_time_ = _info.simTime;
        }
}

void GazeboRosParabolicMotionPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_cmd_vel_ = *_msg;
}
    
void GazeboRosParabolicMotionPrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}
    
void GazeboRosParabolicMotionPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);
  transform_broadcaster_->sendTransform(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosParabolicMotion)
}

