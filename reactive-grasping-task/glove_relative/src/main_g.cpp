//general utilities
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>

// utilities
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#include <IMUGL.h>
#include "listener_phase_space.h"


//-----------------------------------------------------
//                                             	  kbhit
//-----------------------------------------------------
int kbhit(void) 
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}


int main(int argc, char** argv)
{	

	/********************
	*					*
	*	     ROS		*
	*					*
	********************/
	ros::init(argc, argv, "imu_gripper", ros::init_options::NoSigintHandler);
  	ros::NodeHandle n;

	// Parse parameters
  	std::string hand_name = "soft_hand";
  	n.param<std::string>("hand_name", hand_name, "soft_hand");

  	ros::Publisher pub_joints = n.advertise<sensor_msgs::JointState>("joint_states", 1000); 
  	sensor_msgs::JointState my_joint_state;
  
  	//my_joint_state.name.resize(33);
  	//my_joint_state.position.resize(33);

  	// Init position 
  	//for( int j = 0; j < 33; ++j)
  	//{
   	// 	my_joint_state.position[j] = 0;
  	//}

	// Publish first message before looping
	my_joint_state.header.stamp = ros::Time::now();
	pub_joints.publish(my_joint_state);
	ros::spinOnce();


	/********************
	*					*
	*	    IMUGL		*
	*					*
	********************/    
    IMUGL gripper;
    gripper.start();
    gripper.initialOffset();


    /********************
	*					*
	*	     PS  		*
	*					*
	********************/    
    phase_space ps;


	//Press CTRL+C to stop data stream loop
	/********************
	*					*
	*	    LOOP		*
	*					*
	********************/
    double spin_rate = 100;
	ros::Rate rate(spin_rate);

    int handtype = 1;

	while(ros::ok())
	{	
       	
		gripper.computeAngles();
		gripper.printIMUangles();		

		// hit from keyboard
		if (kbhit())
    	{
        	char x = getchar();
        	if(x == 'o')
    			gripper.initialOffset();

    		if(x == 'w')
			{
				std::string s;
				std::cout<<"\n\n\033[1;32m# Experiment \033[0m\n"<<std::endl;
                std::cin >> s;  
                ps.openFile(s);
   			    gripper.openFiles(s);
			}

			if(x == 'c')
            {   
                std::cout << "\033[1;31mclose files \033[0m\n" <<std::endl;
                ps.closeFile();
                gripper.closeFiles();
            }

        }


        // save Data if enabled
        ps.saveData();
        gripper.saveData();

        // pub on rviz
        my_joint_state.header.stamp = ros::Time::now();
		pub_joints.publish( my_joint_state );
		rate.sleep();
		ros::spinOnce();
	}	


	return 0;
}