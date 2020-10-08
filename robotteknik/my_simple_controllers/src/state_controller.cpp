#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

#define KINEMATICS_CLASSES_IMPLEMENTATION
#include <KinematicsClasses.h>

namespace my_simple_controllers {

void StateController::update(const ros::Time& time, const ros::Duration& period) {
   ROS_INFO("State Controller: here read joint handles, compute forward kinematic model and publish tool tip pose to TF");
}

bool StateController::init(hardware_interface::JointStateInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
   ROS_INFO("State Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");

    /*
   Joint angles
   axis = {0, 0, 1} = z-axis
   1: pi/4
   2: -pi/8
   3: -pi/8
   eef_fixed: 0

   Link lengths, x-axis
   base-link: 0
   1: 0.5
   2: 0.3
   3: 0.2
   eef-link: 0

   */

   ROBOTICS_LAB::Link* link_ptr;
   link_ptr = get_next_link(chain, nullptr);

   ROBOTICS_LAB::HomogenousTransform pose_offset = ROBOTICS_LAB::get_identity_matrix();
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);
   ROBOTICS_LAB::set_joint_angle(link_ptr->child_joint, PI/4.0f);

   pose_offset.column[4] = {0.5, 0, 0, 1};
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);
   ROBOTICS_LAB::set_joint_angle(link_ptr->child_joint, -PI/8.0f);
   
   pose_offset.column[4] = {0.3, 0, 0, 1};
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);
   ROBOTICS_LAB::set_joint_angle(link_ptr->child_joint, -PI/8.0f);

   pose_offset.column[4] = {0.2, 0, 0, 1};
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);
   ROBOTICS_LAB::set_joint_angle(link_ptr->child_joint, 0);


   
  
   return true;
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
                       controller_interface::ControllerBase)
