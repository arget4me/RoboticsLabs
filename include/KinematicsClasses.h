#ifndef KINEMATICS_CLASSES_HEADER
#define KINEMATICS_CLASSES_HEADER

#include "HomogenousTransform.h"

namespace ROBOTICS_LAB
{

class Joint;
class Link;
class Chain;



class Joint
{
    std::string name;
    Vec4 axis_angle;
    struct parent
    {
        Link* link;
        HomogenousTransform pose_offset;
    };
};

class Link
{
    std::string name;
    Joint* parent_joint;
    Joint* child_joint;
};

class Chain
{
    std::string name;
    Link base_link;
    struct children
    {
        int count;
        Joint* joints_array; // = new Joints[chain.children.count]; 
        Link* links_array; // = new Joints[chain.children.count];
    };
};

};

//#define KINEMATICS_CLASSES_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef KINEMATICS_CLASSES_IMPLEMENTATION
    
#endif









#endif