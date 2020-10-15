#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <eigen3/Eigen/Dense>

static KDL::TreeFkSolverPos_recursive* fksolver;
static const std::string target_segment = "three_dof_planar_eef";
static KDL::JntArray joints(3);

const KDL::JntArray kdl_frame_to_pose(const KDL::Frame& frame_result)
{
    KDL::JntArray pose(6);
    pose(0) = frame_result.p(0);
    pose(1) = frame_result.p(1);
    pose(2) = frame_result.p(2);

    frame_result.M.GetRPY(pose.data[3], pose.data[4], pose.data[5]);

    return pose;
}

const KDL::JntArray get_current_pose()
{
   KDL::Frame frame_result;
   fksolver->JntToCart(joints, frame_result, target_segment);
   KDL::JntArray current_pose = kdl_frame_to_pose(frame_result);
   
   return current_pose;
}

const void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lab4_node");

    ros::NodeHandle node;

    //geometry_msgs::Twist goal_pose;
    //node.param("goal_pose", goal_pose, {0});

    ros::Subscriber joint_state_subscriber = node.subscribe<sensor_msgs::JointState>("joint_states", 10, joint_states_callback);
    ros::Publisher command_publisher = node.advertise<geometry_msgs::Twist>("command", 10);

    ros::Time previous_time = ros::Time::now();
    ros::Duration fixed_time_step = ros::Duration(0.01);
    while(node.ok())
    {
        if(ros::Time::now() - previous_time >= fixed_time_step)
        {


            previous_time += fixed_time_step;
        }

        ros::spinOnce();
    }

    return 0;
}