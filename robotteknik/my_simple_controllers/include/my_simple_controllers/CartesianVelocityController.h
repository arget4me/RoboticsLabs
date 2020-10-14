#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/tree.hpp>

namespace my_simple_controllers {

  class CartesianVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
    public:
      virtual void update(const ros::Time& time, const ros::Duration& period);

      virtual bool init(hardware_interface::VelocityJointInterface* jvel, 
          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    private:
      hardware_interface::JointStateHandle joint_handles[3];
      KDL::Tree tree;

  };

};

#endif
