#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#define RANDOM_HEADER_IMPLEMENTATION
#include <Random.h>
#include <iostream>


void print_kdl_frame(const KDL::Frame& frame, const std::string& frame_name)
{
    std::cout << "Frame = " << frame_name << "\n[\n";
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
    std::cout << "]\n";
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


    KDL::TreeFkSolverPos_recursive fksolver(tree);
    KDL::JntArray q(tree.getNrOfJoints());

    for(int i = 0; i < tree.getNrOfJoints(); i++)
    {
        float value = randf() * 3.14f * 2.0f;
        q.data[i] = value;
    }
    std::cout << "q = [\n" << q.data << "\n]\n";


    KDL::Frame F_result;

    int i = 0;
    for(std::map<std::string,KDL::TreeElement>::const_iterator s = tree.getSegments().begin(); s != tree.getSegments().end(); s++)
    {
        std::cout << "Segments = " << s->second.segment.getName() << "\n";
    }

    std::string target_segment = "three_dof_planar_eef";

    fksolver.JntToCart(q, F_result, target_segment);

    
    print_kdl_frame(F_result, target_segment);

    /*
    while(nh.ok())
    {

        ros::spinOnce();
    }
    */

    return 0;
}