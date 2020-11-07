#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>

static KDL::TreeFkSolverPos_recursive* fksolver;
static const std::string target_segment = "three_dof_planar_eef";
static KDL::JntArray joints(3);
static bool wait_for_path_request = true;
static KDL::JntArray start_pose(6);
static KDL::JntArray goal_pose(6);
static KDL::JntArray pose_offset(6);
static double length = 1;
static ros::Time previous_time;
static std::vector<KDL::JntArray> planned_trajectory;
static std::vector<KDL::JntArray> trajectory;
static std::vector<KDL::JntArray> joint_trajectory;


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


double trapezoidal_profile(double t, double time_final, double p_start, double p_final, double cruise_vel)
{
    if(time_final < 1e-5)
    {
        return 0.0;
    }

    if(fabs((p_final - p_start) / time_final) >= cruise_vel)
    {
        ROS_ERROR("Cruise speed [%f] too small for distance [%f] with final time [%f]", cruise_vel, fabs(p_final - p_start), time_final);
        return 0.0;
    }

    if(fabs(2 * (p_final - p_start) / time_final) < cruise_vel)
    {
        ROS_ERROR("Cruise speed [%f] too large for distance [%f] with final time [%f]", cruise_vel, fabs(p_final - p_start), time_final);
        return 0.0;
    }

    double time_cruise = (p_start - p_final + cruise_vel * time_final) / cruise_vel;
    double p_acceleration = (cruise_vel * cruise_vel) / (p_start - p_final + cruise_vel * time_final);

    if( t >= 0 && t <= time_cruise)
    {
        //constant acceleration
        return (p_start + 0.5 * p_acceleration * t * t);
    }
    else if (t > time_cruise && t <= time_final - time_cruise)
    {
        //constant velocity : Cruise part
        return (p_start + p_acceleration * time_cruise * (t - time_cruise / 2.0));
    }
    else if(t > time_final - time_cruise && t <= time_final)
    {
        //constant deceleration
        return (p_final - 0.5 * p_acceleration * (time_final - t) * (time_final - t));
    }

    ROS_ERROR("Time t [%f] out of range [0.0, %f]", t, time_final);
    return 0.0;
}

KDL::JntArray rectelinear_path(const KDL::JntArray& start_pose, const KDL::JntArray pose_offset, double length, double s)
{
    KDL::JntArray pose(6);
    pose.data = (start_pose.data + s * pose_offset.data / length);
    return pose;
}


geometry_msgs::Twist get_next_velocity(const KDL::JntArray& start_pose, const KDL::JntArray pose_offset, 
int current_step, int max_nr_steps, double fixed_time_step)
{
    geometry_msgs::Twist p_vel;
    if(current_step >= 0 && current_step < max_nr_steps)
    {
        double d_current_step = (double)current_step;
        double d_max_nr_steps = (double)max_nr_steps;

        double t_prev = (d_current_step - 1) / d_max_nr_steps;
        double t_current = (d_current_step) / d_max_nr_steps;

        double previous_s = trapezoidal_profile(t_prev, 1.0, 0.0, length, length*1.8);
        double current_s = trapezoidal_profile(t_current, 1.0, 0.0, length, length*1.8);

        KDL::JntArray previous_pose = rectelinear_path(start_pose, pose_offset, length, previous_s);
        KDL::JntArray current_pose = rectelinear_path(start_pose, pose_offset, length, current_s);

        //save trajactories
        planned_trajectory.push_back(current_pose);
        trajectory.push_back(get_current_pose());
        joint_trajectory.push_back(joints);

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
    node.param("lab4_node/pose_x", goal_pose.data[0], 0.0);
    node.param("lab4_node/pose_y", goal_pose.data[1], 0.0);
    node.param("lab4_node/pose_z", goal_pose.data[2], 0.05);
    node.param("lab4_node/pose_roll", goal_pose.data[3], 0.0);
    node.param("lab4_node/pose_pitch", goal_pose.data[4], 0.0);
    node.param("lab4_node/pose_yaw", goal_pose.data[5], 0.0);

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
                eef_vel = get_next_velocity(start_pose, pose_offset, current_step++, max_nr_steps, dt);
                command_publisher.publish(eef_vel);
                previous_time += fixed_time_step;
                //ROS_INFO("step: %d", current_step-1);
            }
        }

        ros::spinOnce();
    }


    //save trajactories
    {
        std::ofstream myfile;
        myfile.open ("joint_trajectory.csv");
        myfile << "joint_1" << ",";
        myfile << "joint_2" << ",";
        myfile << "joint_3" << ",";
        for(int i = 0; i < trajectory.size(); i++)
        {
            myfile << "\n";
            myfile << joint_trajectory[i].data[0] << ",";
            myfile << joint_trajectory[i].data[1] << ",";
            myfile << joint_trajectory[i].data[2] << ",";
        }
        myfile.flush();
        myfile.close();
    }

    {
        std::ofstream myfile;
        myfile.open ("trajectory.csv");
        myfile << "x" << ",";
        myfile << "y" << ",";
        myfile << "z" << ",";
        myfile << "roll" << ",";
        myfile << "pitch" << ",";
        myfile << "yaw" << ",";
        for(int i = 0; i < trajectory.size(); i++)
        {
            myfile << "\n";
            myfile << trajectory[i].data[0] << ",";
            myfile << trajectory[i].data[1] << ",";
            myfile << trajectory[i].data[2] << ",";
            myfile << trajectory[i].data[3] << ",";
            myfile << trajectory[i].data[4] << ",";
            myfile << trajectory[i].data[5] << ",";
        }
        myfile.flush();
        myfile.close();
    }

    {
        std::ofstream myfile;
        myfile.open ("planned_trajectory.csv");
        myfile << "x" << ",";
        myfile << "y" << ",";
        myfile << "z" << ",";
        myfile << "roll" << ",";
        myfile << "pitch" << ",";
        myfile << "yaw" << ",";
        for(int i = 0; i < planned_trajectory.size(); i++)
        {
            myfile << "\n";
            myfile << planned_trajectory[i].data[0] << ",";
            myfile << planned_trajectory[i].data[1] << ",";
            myfile << planned_trajectory[i].data[2] << ",";
            myfile << planned_trajectory[i].data[3] << ",";
            myfile << planned_trajectory[i].data[4] << ",";
            myfile << planned_trajectory[i].data[5] << ",";
        }
        myfile.flush();
        myfile.close();
    }



    return 0;
}