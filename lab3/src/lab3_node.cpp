#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lab3_node");
    ros::NodeHandle nh;

    while(nh.ok())
    {

        ros::spinOnce();
    }

    return 0;
}