#ifndef KINEMATICS_CLASSES_HEADER
#define KINEMATICS_CLASSES_HEADER

#include "HomogenousTransform.h"

namespace ROBOTICS_LAB
{

struct Joint;
struct Link;
struct Chain;

struct Joint
{
    std::string name;
    Vec4 axis_angle;
    struct
    {
        Link* link;
        HomogenousTransform pose_offset;
    }parent;

    Joint(): Joint("joint-name"){}
    Joint(std::string name);
};

struct Link
{
    std::string name;
    Joint* parent_joint;
    Joint* child_joint;
    Link(): Link("link-name"){}
    Link(std::string name);

};

struct Chain
{
    std::string name;
    Link base_link;
    struct
    {
        int count;
        Joint* joints_array; // = new Joints[chain.children.count]; 
        Link* links_array; // = new Joints[chain.children.count];
    }children;
    Chain() :Chain("chain", 1) {}
    Chain(std::string name, int num_children);
    ~Chain();
};
Link* get_next_link(Chain& chain, Link* current_link);

//HomogenousTransform calculate_pose(Link* current_link);

};

//#define KINEMATICS_CLASSES_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef KINEMATICS_CLASSES_IMPLEMENTATION

ROBOTICS_LAB::Joint::Joint(std::string name):name(name), axis_angle{1, 0, 0, PI/2}, parent{nullptr, {0}}{}
ROBOTICS_LAB::Link::Link(std::string name):name(name), parent_joint(nullptr), child_joint(nullptr){}

ROBOTICS_LAB::Chain::Chain(std::string name, int num_children) : name(name), base_link("base-link"), children{num_children, new Joint[num_children], new Link[num_children]}
{
    for(int i = 0; i < this->children.count; i++)
    {

    }
}

ROBOTICS_LAB::Chain::~Chain()
{
    delete[] this->children.joints_array;
    delete[] this->children.links_array;
}

ROBOTICS_LAB::Link* ROBOTICS_LAB::get_next_link(Chain& chain, Link* current_link)
{
    int i = 0;
    if(&chain.base_link != current_link)
    {
        for(i = 0; i < chain.children.count; i++)
        {
            if(&chain.children.links_array[i] == current_link)
            {
                i++;
                break;
            }
        }
        
    }

    if(i < chain.children.count)
    {
        return &chain.children.links_array[i];//Next link element in array
    }
    else
    {
        return current_link;
    }
}

/*HomogenousTransform ROBOTICS_LAB::calculate_pose(Link* current_link)
{

}*/

#endif









#endif