#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>

#define RANDOM_HEADER_IMPLEMENTATION
#include <Random.h>
#include <iostream>

static KDL::JntArray* global_joints_ptr = nullptr; 

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

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    for(int i = 0; i < joint_state_msg->position.size(); i++)
    {
        //std::cout << "[" << i << "] : " << joint_state_msg->name[i] << " = " << joint_state_msg->position[i] << "\n";
        if(global_joints_ptr != nullptr)
        {
            (*global_joints_ptr)(i) = (double)joint_state_msg->position[i];
        }
    }
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
    KDL::TreeFkSolverPos_recursive fksolver(tree);
    KDL::JntArray joints(tree.getNrOfJoints());
    global_joints_ptr = &joints;
    KDL::Frame F_result;
    std::string target_segment = "three_dof_planar_eef";

    /*
        for(int i = 0; i < tree.getNrOfJoints(); i++)
        {
            float value = randf() * 3.14f * 2.0f;
            joints(i) = (double)value;
        }
        std::cout << "joints = [\n" << joints.data << "\n]\n";

        for(std::map<std::string,KDL::TreeElement>::const_iterator s = tree.getSegments().begin(); s != tree.getSegments().end(); s++)
        {
            std::cout << "Segments = " << s->second.segment.getName() << "\n";
        }
    
    */
    

    ros::Subscriber joint_states_subscriber = node.subscribe("joint_states", 10, joint_states_callback);

    ros::Time previous_second = ros::Time::now();
    while(node.ok())
    {
        if(ros::Time::now() - previous_second >= ros::Duration(1))
        {

            fksolver.JntToCart(joints, F_result, target_segment);
            print_kdl_frame(F_result, target_segment);

            previous_second += ros::Duration(1);
        }
        ros::spinOnce();
    }

    return 0;
}