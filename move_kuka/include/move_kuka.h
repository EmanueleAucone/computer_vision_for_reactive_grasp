#include <eigen3/Eigen/Eigen>
#include <Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <csignal>
#include <string>
#include <unistd.h>
#include <boost/algorithm/string.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>

#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include "lwr_controllers/SetCartesianImpedanceCommand.h"
//Aucone-Bagheri
#include <lwr_controllers/one_task_inverse_kinematics.h>



class move_kuka
{
public:

	move_kuka();

	~move_kuka();

	//Aucone-Bagheri
	void manager();
	void detection();

	void homePosition();

private:

	ros::NodeHandle n_;
	ros::Subscriber sub_imu_id_;
	//Aucone-Bagheri
	ros::Subscriber sub_contact_detection_;
	std::string pkg_path_;	
	std::string finger_name_;	
	int cg_; // count_grasp_

	int trajectory_type;
	double traj_time;
	double spin_rate;
	double box_size;

	Eigen::Affine3d pose_, pose_init, pose_target;
	lwr_controllers::Stiffness zero_stiffness_;
	geometry_msgs::Wrench zero_wrench_;
	lwr_controllers::CartesianImpedancePoint msg_;

	float degree_;
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
	//Aucone-Bagheri
	ros::Publisher pub_command_,pub_command_t, pub_home_, pub_traj_, pub_flag_;
	float A_,offset_x_,offset_y_,offset_z_,delta_degree_, q_w_, q_x_, q_y_, q_z_; 
	float rest_x_, rest_y_, rest_z_, rest_q_w_, rest_q_x_, rest_q_y_, rest_q_z_;
	float stiffness_t, stiffness_r, damping, wrench; 
	bool ground_;
	double off_high_;
	bool bowl_, sliding_;
	std_msgs::Float64MultiArray joint_home;
	//trajectory_msgs::JointTrajectory joint_traj;

	//Aucone-Bagheri
	//Object params
	float obj_posX, obj_posY, obj_posZ, obj_roll, obj_pitch, obj_yaw, angle_z;
	float dimX, dimY, dimZ;
	float height;
	int height_index;
	geometry_msgs::Pose pose_ee;		//Message to be published

	//Aucone-Bagheri
	float mypose;
	float mypose1, mypose2, mypose3, mypose4, mypose5, mypose6, mydimx, mydimy, mydimz, myangle;
	int myindex;
	Eigen::Matrix3d Q;
	Eigen::Quaterniond quat, q_middle;
	//Eigen::Vector3d vec;

	// void tondoDatabase();
	// void openTondoDatabase(float& x,float& y,float& z,float& angle);

	int interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish, double traj_time_local=2.0);
	void finishPosition(float z);
	//Aucone-Bagheri
	Eigen::Affine3d getCurrentPosition();
	void lateralGrasp();
	void topGrasp();
	void topRightLeftGrasp();
	void bottomGrasp();
	void pinchGrasp();
	void pinchRightLeftGrasp();
	void slideGrasp();
	void flipGrasp();
	void objectRecognition();

	void handClosure(float v);
	ros::Publisher hand_publisher_;

	// call back
	//Aucone-Bagheri
	void callWichFinger(std_msgs::String msg);
	void callImuId(std_msgs::Int64 imu_id);
	void contactDetection(const std_msgs::String& msg);

	//Aucone-Bagheri
	bool flag_contact_ = false;
	bool flag_grasp_ = false;
	bool flag_stop_ = false;
	bool flag_obj_ = false;	

	//Aucone-Bagheri
	enum grasp { lateral, top, topRL, bottom, pinch, pinchRL, slide, flip, obj_recognition };
	grasp g;

};
