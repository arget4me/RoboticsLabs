#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

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
    ROS_INFO(robot_desc_string.c_str());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    KDL::TreeFkSolverPos_recursive fksolver(tree);
    KDL::JntArray q(tree.getNrOfJoints());



    KDL::Frame F_result;
    fksolver.JntToCart(q, F_result, tree.getSegments().end()->second.segment.getName());

    std::cout << "Frame = \n";
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            std::cout << F_result(i,j) << " ";
        }
        std::cout << "\n";
    }


    /*
    while(nh.ok())
    {

        ros::spinOnce();
    }
    */

    return 0;
}