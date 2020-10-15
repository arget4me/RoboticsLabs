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
static bool wait_for_path_request = true;
static KDL::JntArray start_pose(6);
static KDL::JntArray goal_pose(6);
static KDL::JntArray pose_offset(6);
static double length = 1;
static ros::Time previous_time;

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


geometry_msgs::Twist get_next_velocity(int current_step, int max_nr_steps, double fixed_time_step)
{
    geometry_msgs::Twist p_vel; //Initializes with 0 values.
    if(current_step > 0 && current_step < max_nr_steps)
    {
        double d_current_step = (double)current_step;
        double d_max_nr_steps = (double)max_nr_steps;
        double current_s = (d_current_step / d_max_nr_steps) * length;
        double previous_s = ((d_current_step - 1) / d_max_nr_steps) * length;

        KDL::JntArray previous_pose(6);
        KDL::JntArray current_pose(6);
        previous_pose.data = (start_pose.data + previous_s * pose_offset.data / length);
        current_pose.data = (start_pose.data + current_s * pose_offset.data / length);

        KDL::JntArray vel(6);
        vel.data = (current_pose.data - previous_pose.data) / fixed_time_step;

        p_vel.linear.x = vel.data[0];
        p_vel.linear.y = vel.data[1];
        p_vel.linear.z = vel.data[2];
        p_vel.angular.x = vel.data[3];
        p_vel.angular.y = vel.data[4];
        p_vel.angular.z = vel.data[5];
    }

    return p_vel;
}

const void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i = 0; i < msg->position.size(); i++)
    {
        joints.data[i] = msg->position[i];
    }

    if(!wait_for_path_request)
    {
        return;
    }

    start_pose = get_current_pose();
    pose_offset.data = goal_pose.data - start_pose.data;
    length = pose_offset.data.norm();

    ROS_INFO("Start trajectory plan");
    previous_time = ros::Time::now();
    previous_time.sec -= 0.1;

    wait_for_path_request = false;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lab4_node");

    ros::NodeHandle node;
    
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
        ROS_INFO(robot_desc_string.c_str());
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    fksolver = new KDL::TreeFkSolverPos_recursive(tree);

    geometry_msgs::Twist eef_vel;

    //node.param("goal_pose", goal_pose, goal_pose);

    goal_pose.data[0] = 0.6;
    goal_pose.data[1] = 0.2;
    goal_pose.data[2] = 0.05;
    goal_pose.data[5] = 0.0;


    ros::Subscriber joint_state_subscriber = node.subscribe<sensor_msgs::JointState>("joint_states", 10, joint_states_callback);
    ros::Publisher command_publisher = node.advertise<geometry_msgs::Twist>("command", 10);

    
    double dt_seconds = 0.1;

    ros::Duration fixed_time_step = ros::Duration(0.1);
    double dt = double(fixed_time_step.sec) + double(fixed_time_step.nsec)*1e-9;
    int current_step = 0;
    int max_nr_steps = 100;
    while(node.ok())
    {
        if(!wait_for_path_request)
        {
            if(current_step > max_nr_steps)
            {
                ROS_INFO("Trajectory plan done");
                std::cout << "End-effector at pose: \n" << get_current_pose().data << "\n";
                geometry_msgs::Twist vel;
                command_publisher.publish(vel);
                break;
            }

            if(ros::Time::now() - previous_time >= fixed_time_step)
            {
                eef_vel = get_next_velocity(current_step++, max_nr_steps, dt);
                command_publisher.publish(eef_vel);
                previous_time += fixed_time_step;
                //ROS_INFO("step: %d", current_step-1);
            }
        }

        ros::spinOnce();
    }

    return 0;
}