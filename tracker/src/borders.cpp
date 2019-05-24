#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Parameters
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool flag = true;

//pcl::visualization::PCLVisualizer viewer("Rviz");
pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;

// Callback 
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	if(flag)
	{
		// Container for original & filtered data
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2Ptr cloudFilteredPtr(cloud_filtered);

		// Convert to PCL data type
		pcl_conversions::toPCL(*cloud_msg, *cloud);

		// Perform voxel grid downsampling filtering
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud(cloudPtr);
		sor.setLeafSize (0.015, 0.015, 0.015);
		sor.filter(*cloudFilteredPtr);

		pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);

		// convert the pcl::PointCloud2 tpye to pcl::PointCloud2<pcl::PointXYZ>
		pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

		pcl::PointCloud<pcl::PointXYZ>::Ptr final_pointcloud_ptr;
		final_pointcloud_ptr = xyzCloudPtr;

		pcl::PointCloud<pcl::PointXYZ>& final_pointcloud = *final_pointcloud_ptr;

		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; //TODO
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
		scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f(final_pointcloud.sensor_origin_[0],
				                                     	  final_pointcloud.sensor_origin_[1],
				                                     	  final_pointcloud.sensor_origin_[2])) *
			       				  Eigen::Affine3f(final_pointcloud.sensor_orientation_);
	  
		// Create RangeImage from the PointCloud
		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 500;

		pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;   
		range_image.createFromPointCloud(final_pointcloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		//range_image.integrateFarRanges(far_ranges); PAY ATTENTION
		//if (setUnseenToMaxRange)
		//	range_image.setUnseenToMaxRange();

		// Open 3D viewer and add point cloud
		//viewer.setBackgroundColor(1, 1, 1);
		//viewer.addCoordinateSystem(1.0f, "global");
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (final_pointcloud_ptr, 0, 0, 0);
		//viewer.addPointCloud(final_pointcloud_ptr, point_cloud_color_handler, "original point cloud");
	  
		// Extract borders
		pcl::RangeImageBorderExtractor border_extractor(&range_image);
		pcl::PointCloud<pcl::BorderDescription> border_descriptions;
		border_extractor.compute(border_descriptions);
	  
		// Show points in 3D viewer
		pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
				                    veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
				                    shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);

		pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
				              & veil_points = * veil_points_ptr,
				              & shadow_points = *shadow_points_ptr;

		for (int y=0; y< (int)range_image.height; ++y)
		{
			for (int x=0; x< (int)range_image.width; ++x)
			{
				if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
					border_points.points.push_back(range_image.points[y*range_image.width + x]);
				if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
					veil_points.points.push_back(range_image.points[y*range_image.width + x]);
				if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
					shadow_points.points.push_back(range_image.points[y*range_image.width + x]);
			}
		}

		std::cout << border_points.points[0] << std::endl;
/*
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
		viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
		viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
		viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
*/
		// Show points on range image
		range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");

		flag = false;
		//m_pub.publish
	}
}

// Main
int main (int argc, char** argv)
{
	ros::init (argc, argv, "borders");
	ros::NodeHandle m_nh;
	ros::Subscriber m_sub;
	ros::Publisher m_pub;

	// Read point cloud subscribing to the depth camera topic
	m_sub = m_nh.subscribe("/camera/depth_registered/points",1, &cloud_cb);
	//m_pub = m_nh.advertise<pcl::PointWithRange>("tracker/borders",1);
	ros::Rate loop(10);

	// main loop
	while (ros::ok())
	{
		//range_image_borders_widget->spinOnce();
		//viewer.spinOnce();
		ros::spinOnce();
		loop.sleep();
		//pcl_sleep(0.01);
	}
}
