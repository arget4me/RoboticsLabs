#include <my_simple_controllers/CartesianVelocityController.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <iostream>

#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {

const KDL::JntArray kdl_frame_to_pose(const KDL::Frame& frame_result)
{
    KDL::JntArray pose(6);
    pose(0) = frame_result.p(0);
    pose(1) = frame_result.p(1);
    pose(2) = frame_result.p(2);

    frame_result.M.GetRPY(pose.data[3], pose.data[4], pose.data[5]);

    return pose;
}

void CartesianVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
   ROS_INFO("Cartesian Velocity Controller: update here");
   
   /*@TODO: Read from the JointHandle interface values for the 
      current configuration q and configurationspace velocity ˙q
   */
   KDL::JntArray q(3);
   KDL::JntArray q_vel(3);
   for(int i = 0; i < 3; i++)
   {
      q(i) = joint_handles[i].getPosition();
      q_vel(i) = joint_handles[i].getVelocity();
   }
   std::cout << "q =\n" << q.data << "\n q-vel =\n" << q_vel.data << "\n";

   /*@TODO:
   Calculate the Jacobian for the end effector frame J(q).
         @NOTE: needs the tree structure to get the jacobian
   */
   KDL::Frame frame_result;
   KDL::Jacobian jacobian_result(tree.getNrOfJoints());

   //@NOTE: [IMPORTANT] q doesn't have joint names, link the up.

   fksolver->JntToCart(q, frame_result, target_segment);
   KDL::JntArray current_pose = kdl_frame_to_pose(frame_result);

   jacobian_solver->JntToJac(q, jacobian_result, target_segment);
        

   /*TODO:
   Calculate the current desired Cartesian pose p = p(i − 1) + ˙pdt, where ˙p is the
      desired end effector velocity and dt is the control loop time elapsed 
      (the duration parameter of the update function). 

      p(0) is the initial starting pose obtained through the FK model.
      @NOTE: is p(0) calculated once in init(...) and then used all the time or is p(0) the current position.
   */

   /*TODO:
   Calculate the error in desired pose as e = p − FK(q).
   */

   /*TODO:
   Calculate the desired control as u = Ke + ˙p
   */

   /*TODO:
   Calculate the desired joint control as ˙q = invJ(q)u.
   */

   /*TODO:
   Set the desired velocity ˙q to the joint handles
   */

}

bool CartesianVelocityController::init(hardware_interface::VelocityJointInterface* jvel, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
   ROS_INFO("Cartesian Velocity Controller: initialize here");

   std::string robot_desc_string;
   root_nh.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, tree)){
      ROS_INFO(robot_desc_string.c_str());
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

   {
      joint_handles[0] = jvel->getHandle("three_dof_planar_joint1");
      joint_handles[1] = jvel->getHandle("three_dof_planar_joint2");
      joint_handles[2] = jvel->getHandle("three_dof_planar_joint3");
   }

   p_vel = KDL::JntArray(6);

   fksolver = new KDL::TreeFkSolverPos_recursive(tree);
   jacobian_solver = new KDL::TreeJntToJacSolver(tree);

   /*
   @TODO: Expose to the outside a topic on which commanded velocity in Cartesian space
      can be received and assume that the initial command is for zero velocity
   */

   ROS_INFO("Init done!");

   return true;
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::CartesianVelocityController,
                       controller_interface::ControllerBase)
