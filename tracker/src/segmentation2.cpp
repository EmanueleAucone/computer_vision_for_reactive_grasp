#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#include <pcl/registration/icp.h>

bool flag = true;

pcl::PCLPointCloud2* PC = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2* firstPC = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2* secondPC = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2* thirdPC = new pcl::PCLPointCloud2;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPC (new pcl::PointCloud<pcl::PointXYZRGB>());
//Eigen::Matrix3f A;
Eigen::Quaternionf quat, q_middle;
Eigen::Matrix4f transformation;

float home_x = 0.72;
float home_y = 0.0;
float home_z = 1.0;
float mypose1, mypose2, mypose3, mypose4, mypose5, mypose6, mydimx, mydimy, mydimz, myangle;
int myindex;
float obj_posX, obj_posY, obj_posZ, obj_roll, obj_pitch, obj_yaw, angle_z;
float dimX, dimY, dimZ;
int height_index;

// define callback function
//Aucone-Bagheri
void cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
{
	pcl_conversions::toPCL(cloud_msg, *PC);
	std::cout << "Saved!" << std::endl;	
}	

int main (int argc, char** argv)
{	
	// Initialize ROS
	ros::init (argc, argv, "segmentation2");
	ros::NodeHandle m_nh;
	ros::Subscriber m_sub;
	ros::Publisher m_pub1, m_pub2, m_pub3, m_pubf;

	m_sub = m_nh.subscribe ("tracker/pcl_test",1, &cloud_cb);
	m_pub1 = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/final1",1);
	m_pub2 = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/final2",1);
	m_pub3 = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/final3",1);
	m_pubf = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/finalf",1);	

	ros::Rate loop(10);

	// read the info acquired with the segmentation_sim node
	std::ifstream input_file;
	input_file.open ("/home/emanuele/catkin_ws/src/move_kuka/src/my_input_file2.txt");
	while (input_file >> mypose1 >> mypose2 >> mypose3 >> mypose4 >> mypose5 >> mypose6 >> myangle >> mydimx >> mydimy >> mydimz >> myindex) 		{
		obj_posX     = mypose1;
		obj_posY     = mypose2;
		obj_posZ     = mypose3;
		obj_roll     = mypose4;
		obj_pitch    = mypose5;
		obj_yaw      = mypose6;
		angle_z	     = myangle;
		dimX 	     = mydimx;
		dimY 	     = mydimy;
		dimZ 	     = mydimz;	
		height_index = myindex;
	}

	pcl::PCLPointCloud2 outputPCL1, outputPCL2, outputPCL3, outputPCLf;	
	sensor_msgs::PointCloud2 output1, output2, output3, outputf;

	while(ros::ok())
	{
		if(flag)
		{
			std::cout << "\r\n\n\n\033[32m\033[1mClick to save the first point cloud.\033[0m" << std::endl;
			//while ((getchar()) != '\n');			
			getchar();

			ros::spinOnce();				

			*firstPC = *PC;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr1 (new pcl::PointCloud<pcl::PointXYZRGB>); 
			pcl::fromPCLPointCloud2(*firstPC, *xyzCloudPtr1);
		/*
			// rotate the first point cloud
			q_middle = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())	//+180°	
				* Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY())	//+90°
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());	

			quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())		
				* Eigen::AngleAxisf(1.48, Eigen::Vector3f::UnitY())	// just a little bit less than 90°	
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());	
						
			Eigen::Matrix3f A = Eigen::Matrix3f(quat*q_middle);
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					transformation(i,j) = A(i,j);
				}
			}
			transformation(3,0) = 0;
			transformation(3,1) = 0;
			transformation(3,2) = 0;
			transformation(0,3) =  home_x;
			transformation(1,3) =  home_y;
			transformation(2,3) =  home_z;
			transformation(3,3) = 1;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr trasformedPC1 (new pcl::PointCloud<pcl::PointXYZRGB>()); 
			pcl::transformPointCloud(*xyzCloudPtr1, *trasformedPC1, transformation);
		*/
			std::cout << "\r\n\n\n\033[32m\033[1mClick to save the second point cloud.\033[0m" << std::endl;
			//while ((getchar()) != '\n');			
			getchar();

			ros::spinOnce();
				
			*secondPC = *PC;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr2 (new pcl::PointCloud<pcl::PointXYZRGB>); 
			pcl::fromPCLPointCloud2(*secondPC, *xyzCloudPtr2);
		
			// rotate the first point cloud
			quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())		
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())	// just a little bit less than 90°	
				* Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitZ());	//+30°

			Eigen::Matrix3f A = Eigen::Matrix3f(quat);
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					transformation(i,j) = A(i,j);
				}
			}
			transformation(3,0) = 0;
			transformation(3,1) = 0;
			transformation(3,2) = 0;
			transformation(0,3) =  std::max((float)(obj_posX-dimZ-home_x-0.1), (float)0.0);
			transformation(1,3) =  -(dimZ/2 + 0.25);
			transformation(2,3) = 0;
			transformation(3,3) = 1;
			std::cout << A << std::cout;
		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr trasformedPC2 (new pcl::PointCloud<pcl::PointXYZRGB>()); 
			pcl::transformPointCloud(*xyzCloudPtr2, *trasformedPC2, transformation);

			std::cout << "\r\n\n\n\033[32m\033[1mClick to save the third point cloud.\033[0m" << std::endl;
			//while ((getchar()) != '\n');			
			getchar();

			ros::spinOnce();
				
			*thirdPC = *PC;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr3 (new pcl::PointCloud<pcl::PointXYZRGB>); 
			pcl::fromPCLPointCloud2(*thirdPC, *xyzCloudPtr3);
			// rotate the first point cloud
			quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())		
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())	// just a little bit less than 90°	
				* Eigen::AngleAxisf(-M_PI/6, Eigen::Vector3f::UnitZ());	//-30°

			A = Eigen::Matrix3f(quat);
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					transformation(i,j) = A(i,j);
				}
			}
			transformation(3,0) = 0;
			transformation(3,1) = 0;
			transformation(3,2) = 0;
			transformation(0,3) =  std::max((float)(obj_posX-dimZ-home_x-0.1), (float)0.0);
			transformation(1,3) =  dimZ/2 + 0.25;
			transformation(2,3) = 0;
			transformation(3,3) = 1;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr trasformedPC3 (new pcl::PointCloud<pcl::PointXYZRGB>()); 
			pcl::transformPointCloud(*xyzCloudPtr3, *trasformedPC3, transformation);		
	
			// concatenate the three point cloud
			*finalPC = *xyzCloudPtr1;
			*finalPC += *trasformedPC2;
			*finalPC += *trasformedPC3;

			pcl::toPCLPointCloud2(*xyzCloudPtr1, outputPCL1);
			pcl_conversions::fromPCL(outputPCL1, output1);
			
			pcl::toPCLPointCloud2(*trasformedPC2, outputPCL2);
			pcl_conversions::fromPCL(outputPCL2, output2);

			pcl::toPCLPointCloud2(*trasformedPC3, outputPCL3);
			pcl_conversions::fromPCL(outputPCL3, output3);

			pcl::toPCLPointCloud2(*finalPC, outputPCLf);
			pcl_conversions::fromPCL(outputPCLf, outputf);
			
			flag = false;
		}
		// publish the final point cloud of the reconstructed object
		output1.header.frame_id = "/camera_link";
		output1.header.stamp = ros::Time::now();
		m_pub1.publish(output1);

		output2.header.frame_id = "/camera_link";
		output2.header.stamp = ros::Time::now();
		m_pub2.publish(output2);

		output3.header.frame_id = "/camera_link";
		output3.header.stamp = ros::Time::now();
		m_pub3.publish(output3);

		outputf.header.frame_id = "/camera_link";
		outputf.header.stamp = ros::Time::now();
		m_pubf.publish(outputf);

		// Compute principal directions
		Eigen::Vector4f pcaCentroid;
		pcl::compute3DCentroid(*finalPC, pcaCentroid);
		Eigen::Matrix3f covariance;
		computeCovarianceMatrixNormalized(*finalPC, pcaCentroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases

		// Transform the original cloud to the origin where the principal components correspond to the axes.
		Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
		projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
		projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*finalPC, *cloudPointsProjected, projectionTransform);
		// Get the minimum and maximum points of the transformed cloud.
		pcl::PointXYZRGB minPoint, maxPoint;
		pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
		const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

		// Final transform
		const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
	
		float dimX, dimY, dimZ;
		dimX = maxPoint.x - minPoint.x;
		dimY = maxPoint.y - minPoint.y;
		dimZ = maxPoint.z - minPoint.z;

		std::cout << "dimensione x: " << dimX 			 << " "
			  << "dimensione y: " << dimY 			 << " "
			  << "dimensione z: " << dimZ			 << "\n"
			  << "Posizione: (\n" << pcaCentroid             << ")"
			  << std::endl;

		loop.sleep();				
	}
}
