#include <ros/ros.h>

#define HOMOGENOUS_TRANSFORM_IMPLEMENTATION
#include "HomogenousTransform.h"

#define KINEMATICS_CLASSES_IMPLEMENTATION
#include "KinematicsClasses.h"

#include <iostream>

int main()
{
    std::cout << "Lab2" << "\n";
    ROBOTICS_LAB::Chain chain("Chain", 3);
    //chain.children.count = 3;
    //chain.children.links_array = new ROBOTICS_LAB::Link[chain.children.count];
    chain.children.links_array[0].name = std::string("Link-1");
    chain.children.links_array[1].name = std::string("Link-2");
    chain.children.links_array[2].name = std::string("Link-3");

    //chain.children.joints_array = new ROBOTICS_LAB::Joint[chain.children.count];
    chain.children.joints_array[0].name = std::string("Joint-1");
    chain.children.joints_array[1].name = std::string("Joint-2");
    chain.children.joints_array[2].name = std::string("Joint-3");

    ROBOTICS_LAB::Link* current_link = nullptr;
    ROBOTICS_LAB::Link* next_link = &chain.base_link;
    while(current_link != next_link)
    {
        current_link = next_link;
        std::cout << current_link->name << "\n";
        //ROBOTICS_LAB::HomogenousTransform transform = ROBOTICS_LAB::calculate_pose(current_link);


        next_link = ROBOTICS_LAB::get_next_link(chain, current_link);
    }

    return 0;
}