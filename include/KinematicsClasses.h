#ifndef KINEMATICS_CLASSES_HEADER
#define KINEMATICS_CLASSES_HEADER

#include "HomogenousTransform.h"

namespace ROBOTICS_LAB
{

typedef class Joint;
typedef class Link;

class Joint
{
    Vec4 axis_angle;
    struct parent
    {
        Link* link;
        Vec3 position_offset;
        Vec3 rpy_offset;
    };
};

class Link
{
    Joint* parent_joint;
    Joint* child_joint;
};

};

//#define KINEMATICS_CLASSES_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef KINEMATICS_CLASSES_IMPLEMENTATION
    
#endif









#endif