#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

namespace my_simple_controllers {

  class CartesianVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
    public:
      void command_callback(const geometry_msgs::Twist::ConstPtr& msg);
      
      virtual void update(const ros::Time& time, const ros::Duration& period);

      virtual bool init(hardware_interface::VelocityJointInterface* jvel, 
          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

          
    private:
      hardware_interface::JointHandle joint_handles[3];
      KDL::Tree tree;
      KDL::JntArray p_vel;
      double K = 1.0;
      KDL::TreeFkSolverPos_recursive* fksolver;
      KDL::TreeJntToJacSolver* jacobian_solver; 
      const std::string target_segment = "three_dof_planar_eef";
      hardware_interface::VelocityJointInterface* jvel;
      KDL::JntArray desired_pose;
      ros::Subscriber sub_command;


      const KDL::JntArray kdl_frame_to_pose(const KDL::Frame& frame_result) const;
      const KDL::JntArray get_current_pose() const;

  };

};

#endif
