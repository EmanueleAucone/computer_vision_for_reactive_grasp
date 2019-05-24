#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <qb_interface/inertialSensorArray.h>

std::ofstream logFileAcc, logFileGyro;

// Variable of interest

// Callback
void log_data (const qb_interface::inertialSensorArray& accArray)
{
	logFileAcc << accArray.m[0].x << " "
		   << accArray.m[0].y << " "
		   << accArray.m[0].z << " "
		   << accArray.m[1].x << " "
		   << accArray.m[1].y << " "
		   << accArray.m[1].z << " "
		   << accArray.m[2].x << " "
		   << accArray.m[2].y << " "
		   << accArray.m[2].z << " "
		   << accArray.m[3].x << " "
		   << accArray.m[3].y << " "
		   << accArray.m[3].z << " "
		   << accArray.m[4].x << " "
		   << accArray.m[4].y << " "
		   << accArray.m[4].z << " "
		   << accArray.m[5].x << " "
		   << accArray.m[5].y << " "
		   << accArray.m[5].z << " "
		   << accArray.m[6].x << " "
		   << accArray.m[6].y << " "
		   << accArray.m[6].z << "\n";
}

// Callback
void log_data1 (const qb_interface::inertialSensorArray& gyroArray)
{
	
	logFileGyro << gyroArray.m[0].x << " "
		    << gyroArray.m[0].y << " "
		    << gyroArray.m[0].z << " "
		    << gyroArray.m[1].x << " "
		    << gyroArray.m[1].y << " "
		    << gyroArray.m[1].z << " "
		    << gyroArray.m[2].x << " "
	    	    << gyroArray.m[2].y << " "
		    << gyroArray.m[2].z << " "
	    	    << gyroArray.m[3].x << " "
		    << gyroArray.m[3].y << " "
		    << gyroArray.m[3].z << " "
	    	    << gyroArray.m[4].x << " "
		    << gyroArray.m[4].y << " "
		    << gyroArray.m[4].z << " "
		    << gyroArray.m[5].x << " "
		    << gyroArray.m[5].y << " "
		    << gyroArray.m[5].z << " "
		    << gyroArray.m[6].x << " "
		    << gyroArray.m[6].y << " "
		    << gyroArray.m[6].z << "\n";
}

// Main
int main (int argc, char** argv)
{

	ros::init (argc, argv, "log");
  	
	ros::NodeHandle m_nh;
	ros::Subscriber m_log_sub, m_log_sub1;

  	logFileAcc.open("/home/emanuele/logAcc.txt");
	logFileGyro.open("/home/emanuele/logGyro.txt");

	m_log_sub  = m_nh.subscribe ("qb_class_imu/acc",1, &log_data);
	m_log_sub1 = m_nh.subscribe ("qb_class_imu/gyro",1, &log_data1);

	ros::Rate loop(50);

	while(ros::ok())
	{
		ros::spinOnce();
		loop.sleep();
	}

	logFileAcc.close();
	logFileGyro.close();
	std::cout << "Saved!" << std::endl;
	return 0;
}
