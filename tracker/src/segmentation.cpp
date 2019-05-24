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
#include <tracker/SegmentedClustersArray.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

class segmentation {

	public:

		explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

		// define the subscriber and publisher
		m_sub = m_nh.subscribe ("/camera/depth_registered/points",1, &segmentation::cloud_cb, this);
		m_clusterPub = m_nh.advertise<tracker::SegmentedClustersArray> ("tracker/cluster",100);
		m_clusterPub1 = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/pcl_test",1);
		m_clusterPubV = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/pcl_voxel",1);
		m_clusterPubR = m_nh.advertise<sensor_msgs::PointCloud2> ("tracker/pcl_ransac",1);
		m_tracker_pub1 = m_nh.advertise<geometry_msgs::PoseArray> ("tracker/marker",1);

		}

	private:

	ros::NodeHandle m_nh;
	ros::Publisher m_tracker_pub1;
	ros::Subscriber m_sub;
	ros::Publisher m_clusterPub;
	ros::Publisher m_clusterPub1;
	ros::Publisher m_clusterPubV, m_clusterPubR;

	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
//Aucone-Bagheri
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform voxel grid downsampling filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.015, 0.015, 0.015);
	sor.filter (*cloudFilteredPtr);

	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

	// convert the pcl::PointCloud2 tpye to pcl::PointCloud2<pcl::PointXYZRGB>
	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);
	
	sensor_msgs::PointCloud2 outputV;
	pcl::PCLPointCloud2 outputPCLV;
	// convert to pcl::PCLPointCloud2
	pcl::toPCLPointCloud2(*xyzCloudPtr, outputPCLV);

	// Convert to ROS data type
	pcl_conversions::fromPCL(outputPCLV, outputV);

	outputV.header.frame_id = "/camera_color_optical_frame";
	outputV.header.stamp = ros::Time::now();
	m_clusterPubV.publish(outputV);

/*
	//perform passthrough filtering to remove table leg

	// create a pcl object to hold the passthrough filtered results
	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (xyzCloudPtr);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (.5, 1.1);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*xyzCloudPtrFiltered);
*/

	// create a pcl object to hold the ransac filtered results

	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

	// perform ransac planar filtration to remove table top
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
	// Optional
	seg1.setOptimizeCoefficients (true);
	// Mandatory
	seg1.setModelType (pcl::SACMODEL_PLANE);
	seg1.setMethodType (pcl::SAC_RANSAC);
	seg1.setDistanceThreshold (0.015);

	seg1.setInputCloud (xyzCloudPtr);	//xyzCloudPtrFiltered
	seg1.segment (*inliers, *coefficients);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	//extract.setInputCloud (xyzCloudPtrFiltered);
	extract.setInputCloud (xyzCloudPtr);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*xyzCloudPtrRansacFiltered);

	sensor_msgs::PointCloud2 outputR;
	pcl::PCLPointCloud2 outputPCLR;
	// convert to pcl::PCLPointCloud2
	pcl::toPCLPointCloud2(*xyzCloudPtrRansacFiltered, outputPCLR);

	// Convert to ROS data type
	pcl_conversions::fromPCL(outputPCLR, outputR);

	outputR.header.frame_id = "/camera_color_optical_frame";
	outputR.header.stamp = ros::Time::now();
	m_clusterPubR.publish(outputR);

	// perform euclidean cluster segmentation to separate individual objects

	// Create the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (xyzCloudPtrRansacFiltered);

	// create the extraction object for the clusters
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	// specify euclidean cluster parameters
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (15);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (xyzCloudPtrRansacFiltered);
	// exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
	ec.extract (cluster_indices);

	// declare an instance of the SegmentedClustersArray message
	tracker::SegmentedClustersArray CloudClusters;

	// declare the output variable instances
	sensor_msgs::PointCloud2 output;
	pcl::PCLPointCloud2 outputPCL;

	// here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{

		// create a pcl object to hold the extracted cluster
		pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

		//float minX = 10, maxX = 0, minY = 10, maxY = 0, minZ = 10, maxZ = 0;

		// now we are in a vector of indices pertaining to a single cluster.
		// Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
		}

		// Compute principal directions
		Eigen::Vector4f pcaCentroid;
		pcl::compute3DCentroid(*clusterPtr, pcaCentroid);
		Eigen::Matrix3f covariance;
		computeCovarianceMatrixNormalized(*clusterPtr, pcaCentroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases

		// Transform the original cloud to the origin where the principal components correspond to the axes.
		Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
		projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
		projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*clusterPtr, *cloudPointsProjected, projectionTransform);
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
		/*
		std::cout << "eigenvector0: " << eigenVectorsPCA.col(0) << "\n"
					      << "eigenvector1: " << eigenVectorsPCA.col(1) << "\n"
					      << "eigenvector2: " << eigenVectorsPCA.col(2) << std::endl;		
		*/
		geometry_msgs::PoseArray poseArray;
		geometry_msgs::PoseStamped pose, pose1, pose2, pose3, poseDim;

		pose.pose.position.x = pcaCentroid[0];
		pose.pose.position.y = pcaCentroid[1];
		pose.pose.position.z = pcaCentroid[2];
		pose.pose.orientation.w = bboxQuaternion.w();
		pose.pose.orientation.x = bboxQuaternion.x();
		pose.pose.orientation.y = bboxQuaternion.y();
		pose.pose.orientation.z = bboxQuaternion.z();
		poseArray.poses.push_back(pose.pose);
		pose1.pose.position.x = eigenVectorsPCA.col(0).x();
		pose1.pose.position.y = eigenVectorsPCA.col(0).y();
		pose1.pose.position.z = eigenVectorsPCA.col(0).z();
		poseArray.poses.push_back(pose1.pose);
		pose2.pose.position.x = eigenVectorsPCA.col(1).x();
		pose2.pose.position.y = eigenVectorsPCA.col(1).y();
		pose2.pose.position.z = eigenVectorsPCA.col(1).z();
		poseArray.poses.push_back(pose2.pose);
		pose3.pose.position.x = eigenVectorsPCA.col(2).x();
		pose3.pose.position.y = eigenVectorsPCA.col(2).y();
		pose3.pose.position.z = eigenVectorsPCA.col(2).z();
		poseArray.poses.push_back(pose3.pose);
		poseDim.pose.position.x = dimX;
		poseDim.pose.position.y = dimY;
		poseDim.pose.position.z = dimZ;
		poseArray.poses.push_back(poseDim.pose);

		m_tracker_pub1.publish(poseArray);

		// convert to pcl::PCLPointCloud2
		pcl::toPCLPointCloud2(*clusterPtr, outputPCL);

		// Convert to ROS data type
		pcl_conversions::fromPCL(outputPCL, output);

		// add the cluster to the array message
		CloudClusters.clusters.push_back(output);
	}

	output.header.frame_id = "/camera_color_optical_frame";
	output.header.stamp = ros::Time::now();
	m_clusterPub1.publish(output);

	// publish the clusters
	m_clusterPub.publish(CloudClusters);
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segmentation");
	ros::NodeHandle nh;

	segmentation segs(nh);
	ros::Rate loop(10);

	while(ros::ok()){
		ros::spinOnce ();
		loop.sleep();
	}
}
