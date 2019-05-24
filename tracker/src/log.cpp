#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include <kdl/frames.hpp>
#include <math.h>
#include "tf/transform_datatypes.h"
#include <Eigen/Geometry>

std::ofstream logFile;

// Variable of interest
Eigen::Vector3f position;
Eigen::Vector3f camera_pos;
Eigen::Vector3f camera_eig_x;
Eigen::Vector3f camera_eig_y;
Eigen::Vector3f camera_eig_z;
Eigen::Vector3f eig_x, eig_y, eig_z;
Eigen::Vector3f components, v;
float angle_z;
Eigen::Matrix3f R, M, T;

Eigen::Vector3f angles;

// In simulation we consider the camera on the end-effector of the robot (home position)
float off_x = 0.72;
float off_y = 0.0;
float off_z = 1.0;
float dimX, dimY, dimZ;

bool flag;

// Callback
void log_data (const geometry_msgs::PoseArray& poseArray)
{
	//Rotation and traslation
	camera_pos[0] = poseArray.poses[0].position.x;
	camera_pos[1] = poseArray.poses[0].position.y;
	camera_pos[2] = poseArray.poses[0].position.z;

	/* from RVIZ camera to GAZEBO camera
	R0 = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ())
	* Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX());
	*/
	// from camera to vito_anchor
	R = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ())
	* Eigen::AngleAxisf(1.48, Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());

	position = R*camera_pos;

	M = Eigen::Quaternionf(poseArray.poses[0].orientation.x, 
		       poseArray.poses[0].orientation.y, 
		       poseArray.poses[0].orientation.z,
		       poseArray.poses[0].orientation.w);
	T = R*M;

	angles = T.eulerAngles(2, 1, 0);
	int i;
	for(i = 0; i < 3; i++)
	{
		if(angles[i] < 0)
			angles[i] += 2*M_PI;
		if(angles[i] > 2*M_PI)
			angles[i] -= 2*M_PI;
	}

	dimX = poseArray.poses[4].position.x;
	dimY = poseArray.poses[4].position.y;
	dimZ = poseArray.poses[4].position.z;

	// check which dimension is the height
	camera_eig_x[0] = poseArray.poses[1].position.x;
	camera_eig_x[1] = poseArray.poses[1].position.y;
	camera_eig_x[2] = poseArray.poses[1].position.z;

	camera_eig_y[0] = poseArray.poses[2].position.x;
	camera_eig_y[1] = poseArray.poses[2].position.y;
	camera_eig_y[2] = poseArray.poses[2].position.z;

	camera_eig_z[0] = poseArray.poses[3].position.x;
	camera_eig_z[1] = poseArray.poses[3].position.y;
	camera_eig_z[2] = poseArray.poses[3].position.z;

	eig_x = R*camera_eig_x;
	eig_y = R*camera_eig_y;
	eig_z = R*camera_eig_z;
	
	float eigs[3] = {eig_x[2], eig_y[2], eig_z[2]};
		
	float max = 0;
	int height_index;
	for(int i=0; i<3; i++)
	{
		if (eigs[i]>max){
			height_index = i;
			max = eigs[i];
		}
	}

	Eigen::Vector3f v(0, 0, 1);
	components = T*v;
	angle_z = std::atan2(components[1], components[0]);

	logFile << position[0]+off_x << " "
	      << position[1]+off_y << " "
	      << position[2]+off_z << " "
	      << angles[0]	   << " "
	      << angles[1]	   << " "
	      << angles[2]	   << " "
	      << angle_z	   << " "
	      << dimX		   << " "
	      << dimY		   << " "
	      << dimZ		   << " "
	      << height_index	   << std::endl;
	
	flag = false;
	logFile.close();
	std::cout << "Saved!" << std::endl;
	}

// Main
int main (int argc, char** argv)
{

	ros::init (argc, argv, "log");
	flag = true;
	ros::NodeHandle m_nh;
	ros::Subscriber m_log_sub;

	logFile.open("/home/emanuele/catkin_ws/src/move_kuka/src/my_input_file2.txt");

	m_log_sub = m_nh.subscribe ("tracker/marker",1, &log_data);

	ros::Rate loop(1);

	while(ros::ok() && flag)
	{
		ros::spinOnce ();
		loop.sleep();
	}
	return 0;
}
