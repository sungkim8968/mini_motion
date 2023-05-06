#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_

#include "gazebo/common/Plugin.hh"
#include <memory>

namespace gazebo_plugins
{
    class GazeboRosParabolicMotionPrivate;
    class GazeboRosParabolicMotion : public gazebo::ModelPlugin
    {
        public:
            GazeboRosParabolicMotion();
            ~GazeboRosParabolicMotion();
        protected:
            void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
            void Reset() override;

        private:
            std::unique_ptr<GazeboRosParabolicMotionPrivate> impl_;
    };

}

#endif