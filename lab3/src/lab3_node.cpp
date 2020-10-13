#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>

#define RANDOM_HEADER_IMPLEMENTATION
#include <Random.h>
#include <iostream>

static KDL::JntArray* global_joints_ptr = nullptr; 
static std::string target_segment = "three_dof_planar_eef";
static sensor_msgs::JointState joint_states;

void print_kdl_frame(const KDL::Frame& frame, const std::string& frame_name)
{
    ROS_INFO("\nFrame = %s \n[", frame_name.c_str());
    for(int i = 0; i < 4; i++)
    {
        std::cout << "  ";
        for(int j = 0; j < 4; j++)
        {
            std::cout << frame(i,j);
            if(j % 4 != 3)
            {
                std::cout << ",  ";
            }
        }
        std::cout << "\n";
    }
    std::cout << "]\n\n";
}

void print_kdl_jacobian(const KDL::Jacobian& jacobian, const std::string& frame_name)
{
    ROS_INFO("\nJacobian = %s \n[", frame_name.c_str());
    for(int i = 0; i < 6; i++)
    {
        std::cout << "  ";
        for(int j = 0; j < jacobian.columns(); j++)
        {
            std::cout << jacobian(i,j);
            if(j % jacobian.columns() != jacobian.columns()-1)
            {
                std::cout << ",  ";
            }
        }
        std::cout << "\n";
    }
    std::cout << "]\n\n";
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    static bool once = true;
    if(once)
    {
        joint_states = *joint_state_msg;
        once = false;
    }
    for(int i = 0; i < joint_state_msg->position.size(); i++)
    {
        //std::cout << "[" << i << "] : " << joint_state_msg->name[i] << " = " << joint_state_msg->position[i] << "\n";
        if(global_joints_ptr != nullptr)
        {
            (*global_joints_ptr)(i) = (double)joint_state_msg->position[i];
        }
    }
}

const KDL::JntArray kdl_frame_to_pose(const KDL::Frame& frame_result)
{
    KDL::JntArray pose(6);
    pose(0) = frame_result.p(0);
    pose(1) = frame_result.p(1);
    pose(2) = frame_result.p(2);

    frame_result.M.GetRPY(pose.data[3], pose.data[4], pose.data[5]);

    return pose;
}

void publish_joint_state(const ros::Publisher& joint_states_publisher, const KDL::JntArray& joints)
{
    for(int i = 0; i < joint_states.position.size(); i++)
    {
        joint_states.position[i] = joints(i);
    }
    joint_states.header.stamp = ros::Time::now();

    joint_states_publisher.publish(joint_states);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
}

bool calculate_IK(const KDL::JntArray& joints, const KDL::JntArray& goal, const KDL::Tree& tree, const ros::Publisher& joint_states_publisher)
{
    KDL::JntArray guess = joints;


    bool converged = false;
    KDL::Frame frame_result;
    KDL::Jacobian jacobian_result(tree.getNrOfJoints());

    KDL::TreeFkSolverPos_recursive fksolver(tree);
    KDL::TreeJntToJacSolver jacobian_solver(tree);

    int i = 0;
    double prev_error = 100000.0;
    double lambda = 0.25;
    
    while(!converged && (i++ < 200))
    {
        fksolver.JntToCart(guess, frame_result, target_segment);
        //print_kdl_frame(frame_result, target_segment);
        KDL::JntArray current_pose = kdl_frame_to_pose(frame_result);
        std::cout << "\nCurrent: \n" << current_pose.data;
        std::cout << "\nGoal: \n" << goal.data;

        auto offset = goal.data - current_pose.data;
        std::cout << "\noffset = \n" << offset << "\n";

        if(prev_error < offset.norm())
        {
            ROS_ERROR("Error increasing! %f > %f", offset.norm(), prev_error);
            //break;
        }
        prev_error = offset.norm();

        if(offset.norm() <= 1e-2)
        {
            converged = true;
            break;
        }
        jacobian_solver.JntToJac(guess, jacobian_result, target_segment);
        //print_kdl_jacobian(jacobian_result, target_segment);

        auto joint_offset =  lambda * jacobian_result.data.transpose() * (offset / offset.norm());
        
        std::cout << "\n Joint offset = \n" << joint_offset << "\n\n";

        guess.data += joint_offset;

        std::cout << "[" << i <<"] Guess = \n" << guess.data << "\n\n";

        publish_joint_state(joint_states_publisher, guess);
    }


    return converged;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lab3_node");
    ros::NodeHandle node;

    /*
        kdl_parser/Tutorials/Start using the KDL parser
        2.2 From the parameter server
        http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
    */
    KDL::Tree tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
        ROS_INFO(robot_desc_string.c_str());
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }    

    /*
        fk solver example
        https://www.orocos.org/wiki/Kinematic_and_Dynamic_Solvers.html

        API examples
        https://www.orocos.org/kdl/examples.html

    */
    KDL::Frame frame_result;
    KDL::TreeFkSolverPos_recursive fksolver(tree);
    KDL::JntArray joints(tree.getNrOfJoints());
    global_joints_ptr = &joints;
    

    KDL::Jacobian jacobian_result(tree.getNrOfJoints());
    KDL::TreeJntToJacSolver jacobian_solver(tree);
    

    ros::Subscriber joint_states_subscriber = node.subscribe("joint_states", 10, joint_states_callback);
    ros::Publisher joint_states_publisher = node.advertise<sensor_msgs::JointState>("joint_states", 10);

    KDL::JntArray pose(6);
    node.param("lab3_node/pose_x", pose.data[0], 0.0);
    node.param("lab3_node/pose_y", pose.data[1], 0.0);
    node.param("lab3_node/pose_z", pose.data[2], 0.05);
    node.param("lab3_node/pose_roll", pose.data[3], 0.0);
    node.param("lab3_node/pose_pitch", pose.data[4], 0.0);
    node.param("lab3_node/pose_yaw", pose.data[5], 0.0);

    ros::Time previous_second = ros::Time::now();
    while(node.ok())
    {
        if(ros::Time::now() - previous_second >= ros::Duration(1))
        {

            fksolver.JntToCart(joints, frame_result, target_segment);
            //print_kdl_frame(frame_result, target_segment);

            /*
                http://docs.ros.org/jade/api/orocos_kdl/html/treejnttojacsolver_8cpp_source.html
                Base-frame of the tree is the base of the jacobian

                What effect does the change of base have:
                The base of the jacobian works in the a similar way to the base of a transform matrix.
                In the case of the Jacobian, it expresses "velocity". It means that if in the case of a planar
                arm, if the base-frame and the base of the jacobian are oriented in the same way the jacobian will
                be the same for the positional part. the angular part is always the same since it's planar, and will only
                rotate around one axis.
            */
            jacobian_solver.JntToJac(joints, jacobian_result, target_segment);
            //print_kdl_jacobian(jacobian_result, target_segment);

            //EIGEN MATRIX: //jacobian_result.data;

            calculate_IK(joints, pose, tree, joint_states_publisher);
            break;
            previous_second += ros::Duration(1);
        }
        ros::spinOnce();
    }

    return 0;
}