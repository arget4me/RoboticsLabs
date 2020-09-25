#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define HOMOGENOUS_TRANSFORM_IMPLEMENTATION
#include <HomogenousTransform.h>
#include <iostream>
#include <cstdlib>
#include <time.h>

inline float randf()
{
	srand(time(NULL)*time(NULL));
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2.0f - 1.0f;
}

int main(int argc, char** argv)
{
	ROBOTICS_LAB::test_homogenous_transform();

	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;



	
	ROBOTICS_LAB::Vec4 f1_xyz = {randf(), randf(), randf(), 1};

	ROBOTICS_LAB::HomogenousTransform f1_transform = ROBOTICS_LAB::get_transform_from_eulerZYX({randf(), randf(), randf()});
	f1_transform.column[3] = f1_xyz;
	ROBOTICS_LAB::Vec4 f1_quat = {0};
	ROBOTICS_LAB::get_quaternion_from_transform(&f1_quat, f1_transform);
	
	ROBOTICS_LAB::Vec4 f2_xyz = {randf(), randf(), randf(), 1};

	ROBOTICS_LAB::Vec4 f2_quat = {0};
	ROBOTICS_LAB::HomogenousTransform f2_transform = ROBOTICS_LAB::get_transform_from_eulerZYX({randf(), randf(), randf()});
	f2_transform.column[3] = f2_xyz;
	ROBOTICS_LAB::get_quaternion_from_transform(&f2_quat, f2_transform);

	ROBOTICS_LAB::HomogenousTransform f2_in_f1_transform = ROBOTICS_LAB::apply_transform(f1_transform, f2_transform);
	ROBOTICS_LAB::Vec4 f2_in_f1_axis_angle;
	ROBOTICS_LAB::Vec4 f2_in_f1_quat;
	ROBOTICS_LAB::Vec4& f2_in_f1_xyz = f2_in_f1_transform.column[3];
	ROBOTICS_LAB::get_angle_axis_from_transform(&f2_in_f1_axis_angle, f2_in_f1_transform);
	ROBOTICS_LAB::get_quaternion_from_angle_axis(&f2_in_f1_quat, f2_in_f1_axis_angle);

/*
	std::cout << "F1 = ";
	ROBOTICS_LAB::print_homogenous_transform(f1_transform);
	std::cout << "F2 = ";
	ROBOTICS_LAB::print_homogenous_transform(f2_transform);
	std::cout << "F1_2 = ";
	ROBOTICS_LAB::print_homogenous_transform(f2_in_f1_transform);
*/

	ROBOTICS_LAB::Vec3 samples_pos[10] = {0};
	ROBOTICS_LAB::Vec4 samples_quat[10] = {0};
	ROBOTICS_LAB::Vec4 f2_axis_angle = {0};
	ROBOTICS_LAB::get_angle_axis_from_transform(&f2_axis_angle, f2_transform);
	for(int i = 0; i < 10; i++)
	{
		//starting at 0, ending at a. (a is at the 10th position but after the 9th increment) 
		samples_pos[i].x = f2_xyz.x / 9.0f * i;
		samples_pos[i].y = f2_xyz.y / 9.0f * i;
		samples_pos[i].z = f2_xyz.z / 9.0f * i;
		ROBOTICS_LAB::get_quaternion_from_angle_axis(&(samples_quat[i]), {f2_axis_angle.x, f2_axis_angle.y, f2_axis_angle.z, f2_axis_angle.w / 9.0f * i});
	}

	const char* f1_animation_names[10] =
	{
		"F1_01",
		"F1_02",
		"F1_03",
		"F1_04",
		"F1_05",
		"F1_06",
		"F1_07",
		"F1_08",
		"F1_09",
		"F1_10",
	};
	 
	int loops = 0;
	while(n.ok()){
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(f1_quat.x, f1_quat.y, f1_quat.z, f1_quat.w), tf::Vector3(f1_xyz.x, f1_xyz.y, f1_xyz.z)),
			ros::Time::now(),"base_link", "Frame 1"));
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(f2_quat.x, f2_quat.y, f2_quat.z, f2_quat.w), tf::Vector3(f2_xyz.x, f2_xyz.y, f2_xyz.z)),
			ros::Time::now(),"Frame 1", "Frame 1-2"));
		

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(f2_in_f1_quat.x, f2_in_f1_quat.y, f2_in_f1_quat.z, f2_in_f1_quat.w), tf::Vector3(f2_in_f1_xyz.x, f2_in_f1_xyz.y, f2_in_f1_xyz.z)),
			ros::Time::now(),"base_link", "Frame 0-1-2"));



		for(int i = 0; i < 10; i++)
		{

			broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(samples_quat[i].x, samples_quat[i].y, samples_quat[i].z, samples_quat[i].w), tf::Vector3(samples_pos[i].x, samples_pos[i].y, samples_pos[i].z)),
				ros::Time::now(),"Frame 1", f1_animation_names[i]));
				r.sleep();

		}


		r.sleep();
		if(loops % 100 == 0)
		{
			//std::cout <<"[" << loops / 100 << "]\tPublishing TF...\n";
		}
		loops++;
	}

	return 0;
}
