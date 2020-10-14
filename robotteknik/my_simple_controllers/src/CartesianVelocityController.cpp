#include <my_simple_controllers/CartesianVelocityController.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <iostream>

#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {

const KDL::JntArray CartesianVelocityController::kdl_frame_to_pose(const KDL::Frame& frame_result) const
{
    KDL::JntArray pose(6);
    pose(0) = frame_result.p(0);
    pose(1) = frame_result.p(1);
    pose(2) = frame_result.p(2);

    frame_result.M.GetRPY(pose.data[3], pose.data[4], pose.data[5]);

    return pose;
}

const KDL::JntArray CartesianVelocityController::get_current_pose() const
{
   KDL::JntArray q(3);
   for(int i = 0; i < 3; i++)
   {
      
      q(i) = joint_handles[i].getPosition();
   }

   KDL::Frame frame_result;
   fksolver->JntToCart(q, frame_result, target_segment);
   KDL::JntArray current_pose = kdl_frame_to_pose(frame_result);
   
   return current_pose;
}



void CartesianVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
   if(p_vel.data.norm() <= 1e-4)
   {
      for(int i = 0; i < 3; i++)
      {
         joint_handles[i].setCommand(0);
      }
      return;
   }
   //ROS_INFO("Cartesian Velocity Controller: update here");
   
   /*@TODO: Read from the JointHandle interface values for the 
      current configuration q and configurationspace velocity ˙q
   */
   KDL::JntArray q(3);
   KDL::JntArray q_vel(3);
   for(int i = 0; i < 3; i++)
   {
      q(i) = joint_handles[i].getPosition();
   }

   /*@TODO:
   Calculate the Jacobian for the end effector frame J(q).
         @NOTE: needs the tree structure to get the jacobian
   */
   KDL::Jacobian jacobian_result(tree.getNrOfJoints());
   jacobian_solver->JntToJac(q, jacobian_result, target_segment);

   /*TODO:
   Calculate the current desired Cartesian pose p = p(i − 1) + ˙pdt, where ˙p is the
      desired end effector velocity and dt is the control loop time elapsed 
      (the duration parameter of the update function). 

      p(0) is the initial starting pose obtained through the FK model.
      @NOTE: is p(i-1) the current FK(q) pose??.
   */
   double dt = double(period.sec) + double(period.nsec)*1e-9;
   
   desired_pose.data = desired_pose.data + p_vel.data * dt;

   /*TODO:
   Calculate the error in desired pose as e = p − FK(q).
   */
   KDL::JntArray current_pose = get_current_pose();

   KDL::JntArray error(6);
   error.data = desired_pose.data - current_pose.data;

   /*TODO:
   Calculate the desired control as u = Ke + ˙p
   */
   KDL::JntArray control(6);
   control.data = K * error.data + p_vel.data; 


   /*TODO:
   Calculate the desired joint control as ˙q = invJ(q)u.
   */
   auto psuedo_inverse = (jacobian_result.data.transpose() * jacobian_result.data).inverse() * jacobian_result.data.transpose();
   q_vel.data = psuedo_inverse * control.data;

   /*TODO:
   Set the desired velocity ˙q to the joint handles
   */
   for(int i = 0; i < 3; i++)
   {
      q.data[i] += q_vel.data[i] * dt*10;
      if(q.data[i] <= -1.9 || q.data[i] >= 1.9)
      {
         //stop when a joint has reached it's limit : Set end-effector velocity to 0;
         ROS_ERROR("STOPPING!\nJoint [%d] will pass joint limit [-1.9, 1.9], angle would be = %f", i, q.data[i]);
         for(int x = 0; x < 6; x++)
         {
            if(x < 3)
            {
               q_vel.data[x] = 0.0;
            }
            p_vel.data[x] = 0.0;
         }
         break;
      }
   }

   for(int i = 0; i < 3; i++)
   {
      
      joint_handles[i].setCommand(q_vel.data[i]);
   }

}

void CartesianVelocityController::command_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   for(int i = 0; i < 3; i++)
   {
      p_vel.data[i] = ((double*)&msg->linear)[i];
   }
   for(int i = 0; i < 3; i++)
   {
      p_vel.data[3 + i] = ((double*)&msg->angular)[i];
   }

   ROS_INFO("New velocity registered");
   std::cout << p_vel.data << "\n";

   desired_pose = get_current_pose();
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

   controller_nh.param("three_dof_planar_joint1/p", K, 1.0);

   this->jvel = jvel;

   p_vel = KDL::JntArray(6);

   fksolver = new KDL::TreeFkSolverPos_recursive(tree);
   jacobian_solver = new KDL::TreeJntToJacSolver(tree);

   desired_pose = get_current_pose();


   /*
   @TODO: Expose to the outside a topic on which commanded velocity in Cartesian space
      can be received and assume that the initial command is for zero velocity
   */
  sub_command = root_nh.subscribe<geometry_msgs::Twist>("command", 10, &CartesianVelocityController::command_callback, this);

   ROS_INFO("Init done!");

   return true;
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::CartesianVelocityController,
                       controller_interface::ControllerBase)
