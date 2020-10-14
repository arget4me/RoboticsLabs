#define HOMOGENOUS_TRANSFORM_IMPLEMENTATION
#include <HomogenousTransform.h>

#define KINEMATICS_CLASSES_IMPLEMENTATION
#include <KinematicsClasses.h>

#include <tf/transform_broadcaster.h>
#include <my_simple_controllers/state_controller.h>

#include <iostream>

#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {

void StateController::update(const ros::Time& time, const ros::Duration& period) {
   ROS_INFO("State Controller: here read joint handles, compute forward kinematic model and publish tool tip pose to TF");

   ROBOTICS_LAB::Link* link_ptr = &chain.base_link;   
   for(int i = 0; i < 3; i++)
   {
      ROBOTICS_LAB::set_joint_angle(link_ptr->child_joint, joint_handles[i].getPosition());
      link_ptr = get_next_link(chain, link_ptr);
   }

   {
      ROBOTICS_LAB::HomogenousTransform to_end_effektor = calculate_pose(link_ptr);
      ROBOTICS_LAB::Vec3& eef_xyz = to_end_effektor.column[3].to_vec3;
      ROBOTICS_LAB::Vec4 eef_quat;
      ROBOTICS_LAB::get_quaternion_from_transform(&eef_quat, to_end_effektor);
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(eef_quat.x, eef_quat.y, eef_quat.z, eef_quat.w), tf::Vector3(eef_xyz.x, eef_xyz.y, eef_xyz.z)),
            time,"three_dof_planar_link0", "three_dof_planar_eef"));
   }
   
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

   ROBOTICS_LAB::HomogenousTransform pose_offset;
   ROBOTICS_LAB::HomogenousTransform joint_frame_rotation;
   
   
   ROBOTICS_LAB::Link* link_ptr = &chain.base_link;
   pose_offset = ROBOTICS_LAB::get_identity_matrix();
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);


   link_ptr = get_next_link(chain, link_ptr);
   joint_frame_rotation = ROBOTICS_LAB::get_transform_from_eulerZYX({0, 0, PI_CONSTANT/4});
   pose_offset = ROBOTICS_LAB::get_identity_matrix(); pose_offset.column[3] = {0.5, 0, 0, 1};
   pose_offset = ROBOTICS_LAB::apply_transform(joint_frame_rotation, pose_offset);
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);
   
   link_ptr = get_next_link(chain, link_ptr);
   joint_frame_rotation = ROBOTICS_LAB::get_transform_from_eulerZYX({0, 0, -PI_CONSTANT/8});
   pose_offset = ROBOTICS_LAB::get_identity_matrix(); pose_offset.column[3] = {0.3, 0, 0, 1};
   pose_offset = ROBOTICS_LAB::apply_transform(joint_frame_rotation, pose_offset);
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);

   link_ptr = get_next_link(chain, link_ptr);
   joint_frame_rotation = ROBOTICS_LAB::get_transform_from_eulerZYX({0, 0, -PI_CONSTANT/8});
   pose_offset = ROBOTICS_LAB::get_identity_matrix(); pose_offset.column[3] = {0.2, 0, 0, 1};
   pose_offset = ROBOTICS_LAB::apply_transform(joint_frame_rotation, pose_offset);
   ROBOTICS_LAB::set_link_offset(link_ptr, pose_offset);

   {
      joint_handles[0] = hw->getHandle("three_dof_planar_joint1");
      joint_handles[1] = hw->getHandle("three_dof_planar_joint2");
      joint_handles[2] = hw->getHandle("three_dof_planar_joint3");
   }

   {
      ROBOTICS_LAB::HomogenousTransform to_end_effektor = calculate_pose(link_ptr);
      ROBOTICS_LAB::Vec3& eef_xyz = to_end_effektor.column[3].to_vec3;
      ROBOTICS_LAB::Vec4 eef_quat;
      ROBOTICS_LAB::get_quaternion_from_transform(&eef_quat, to_end_effektor);
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(eef_quat.x, eef_quat.y, eef_quat.z, eef_quat.w), tf::Vector3(eef_xyz.x, eef_xyz.y, eef_xyz.z)),
            ros::Time::now(),"three_dof_planar_link0", "three_dof_planar_eef"));
   }
  
   ROS_INFO("Init done!");

   return true;
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
                       controller_interface::ControllerBase)