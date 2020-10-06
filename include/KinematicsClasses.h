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
    Joint base_joint;
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

inline void set_joint_angle(Joint* joint, float angle);

inline void set_link_offset(Link* link, HomogenousTransform pose_offset);

Link* get_next_link(Chain& chain, Link* current_link);

HomogenousTransform calculate_pose(Link* current_link);

};

//#define KINEMATICS_CLASSES_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef KINEMATICS_CLASSES_IMPLEMENTATION

ROBOTICS_LAB::Joint::Joint(std::string name):name(name), axis_angle{0, 0, 1, 0}, parent{nullptr, get_identity_matrix()}{}
ROBOTICS_LAB::Link::Link(std::string name):name(name), parent_joint(nullptr), child_joint(nullptr){}

ROBOTICS_LAB::Chain::Chain(std::string name, int num_children) : name(name), base_link("base-link"), base_joint("base-joint"), children{num_children, new Joint[num_children], new Link[num_children]}
{
    this->base_link.child_joint = &this->base_joint;
    this->base_joint.parent.link = &this->base_link;
    this->children.links_array[0].parent_joint = &this->base_joint;

    for(int i = 1; i < this->children.count; i++)
    {
        this->children.links_array[i].parent_joint = &this->children.joints_array[i-1];
    }
    for(int i = 0; i < this->children.count; i++)
    {
        this->children.links_array[i].child_joint = &this->children.joints_array[i];
        this->children.joints_array[i].parent.link = &this->children.links_array[i];
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

inline void ROBOTICS_LAB::set_joint_angle(Joint* joint, float angle) {
    if(joint != nullptr)
    {
        joint->axis_angle.data[3] = angle;
    }
}

inline void ROBOTICS_LAB::set_link_offset(Link* link, HomogenousTransform pose_offset) {
    if(link != nullptr && link->child_joint != nullptr)
    {
        link->child_joint->parent.pose_offset = pose_offset;
    }
}

ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::calculate_pose(Link* current_link)
{
    if(current_link == nullptr || current_link->child_joint == nullptr)
    {
        return get_identity_matrix();
    }

    //std::cout << "\nCurrent_Link " << current_link->name << "\n";
    //std::cout << "Child_joint " << current_link->child_joint->name << "\n";

    if(current_link->parent_joint != nullptr)
    {
        HomogenousTransform joint_transform = get_transform_from_angle_axis(current_link->parent_joint->axis_angle);
        HomogenousTransform link_transform = apply_transform(joint_transform, current_link->child_joint->parent.pose_offset);    
        
        HomogenousTransform parent_link_transform = calculate_pose(current_link->parent_joint->parent.link);
        return apply_transform(parent_link_transform, link_transform); 
    }
    else
    {
        return current_link->child_joint->parent.pose_offset;
    }
}

#endif









#endif