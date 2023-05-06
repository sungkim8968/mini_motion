#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_MINI_MOTION_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_MINI_MOTION_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosMiniMotionPrivate;

/// Publish the state of joints in simulation to a given ROS topic.
/**
  \details If the joint contains more than one axis, only the state of the first axis is reported.

  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_joint_state_publisher"
        filename="libgazebo_ros_joint_state_publisher.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/ny_namespace</namespace>

        <!-- Remap the default topic -->
        <remapping>joint_states:=my_joint_states</remapping>

      </ros>

      <!-- Update rate in Hertz -->
      <update_rate>2</update_rate>

      <!-- Name of joints in the model whose states will be published. -->
      <joint_name>left_wheel</joint_name>
      <joint_name>right_wheel</joint_name>
      <joint_name>elbow</joint_name>
      <joint_name>shoulder</joint_name>

    </plugin>
  \endcode
*/
class GazeboRosMiniMotion : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosMiniMotion();

  /// Destructor
  ~GazeboRosMiniMotion();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Callback to be called at every simulation iteration.
  /// Private data pointer
  std::unique_ptr<GazeboRosMiniMotionPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_STATE_PUBLISHER_HPP_
