#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "../include/mini_motion.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosMiniMotionPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /// Joints being tracked.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosMiniMotion::GazeboRosMiniMotion()
: impl_(std::make_unique<GazeboRosMiniMotionPrivate>())
{
}

GazeboRosMiniMotion::~GazeboRosMiniMotion()
{
}

void GazeboRosMiniMotion::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Plugin missing <joint_name>s");
    impl_->ros_node_.reset();
    return;
  }

  sdf::ElementPtr joint_elem = sdf->GetElement("joint_name");
  while (joint_elem) {
    auto joint_name = joint_elem->Get<std::string>();

    auto joint = model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
    } else {
      impl_->joints_.push_back(joint);
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Going to publish joint [%s]",
        joint_name.c_str() );
    }

    joint_elem = joint_elem->GetNextElement("joint_name");
  }

  if (impl_->joints_.empty()) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "No joints found.");
    impl_->ros_node_.reset();
    return;
  }

  // Update rate
  double update_rate = 100.0;
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f", update_rate);
  } else {
    update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->last_update_time_ = model->GetWorld()->SimTime();

  // Joint state publisher
  impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMiniMotionPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosMiniMotionPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  // Populate message
  sensor_msgs::msg::JointState joint_state;
  // 调整大小
  joint_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  joint_state.name.resize(joints_.size());
  joint_state.position.resize(joints_.size());
  joint_state.velocity.resize(joints_.size());

  for (unsigned int i = 0; i < joints_.size(); ++i) {
    auto joint = joints_[i];
    double velocity = joint->GetVelocity(0);
    double position = joint->Position(0);
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = position;
    joint_state.velocity[i] = velocity;
  }
  // Publish
  joint_state_pub_->publish(joint_state);
  // Update time
  last_update_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMiniMotion)
}  // namespace gazebo_plugins
