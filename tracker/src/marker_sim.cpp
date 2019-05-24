#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

double points[12];

// callback
void arrow_cb (const geometry_msgs::PoseArray& poseArray)
{
	int i, j;
	for(i = 0; i < 4; ++i)
	{
		points[0+i*3] = poseArray.poses[i].position.x;
		points[1+i*3] = poseArray.poses[i].position.y;
		points[2+i*3] = poseArray.poses[i].position.z;
	}
}

int main (int argc, char** argv)
{

	// Initialize ROS
	ros::init (argc, argv, "marker_sim");

	ros::NodeHandle m_nh;
	ros::Publisher m_marker_pub;
	ros::Subscriber m_marker_sub1;

	ros::Rate loop(10);

	// define the subscriber and publisher
	m_marker_sub1 = m_nh.subscribe ("tracker/marker",1, &arrow_cb);
	m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("tracker/visualization_marker1",1);

	while(ros::ok())
	{
		 visualization_msgs::MarkerArray marker_array;
		 geometry_msgs::Point p, p1, p2, p3;

		 marker_array.markers.resize(3);

		 // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		 marker_array.markers[0].header.frame_id = "/camera_link";
		 marker_array.markers[0].header.stamp = ros::Time::now();

		 // Set the namespace and id for this marker.  This serves to create a unique ID
		 // Any marker sent with the same namespace and id will overwrite the old one
		 marker_array.markers[0].ns = "marker1";
		 marker_array.markers[0].id = 0;

		 // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		 // marker.type = visualization_msgs::Marker::ARROW;
		 marker_array.markers[0].type = visualization_msgs::Marker::ARROW;

		 // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		 marker_array.markers[0].action = visualization_msgs::Marker::ADD;

		 // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		 p.x = points[0];
		 p.y = points[1];
		 p.z = points[2];
		 marker_array.markers[0].points.push_back(p);
		 p1.x = p.x + points[3];
		 p1.y = p.y + points[4];
		 p1.z = p.z + points[5];
		 marker_array.markers[0].points.push_back(p1);

		 // Set the scale of the marker -- 1x1x1 here means 1m on a side
		 marker_array.markers[0].scale.x = 0.01;
		 marker_array.markers[0].scale.y = 0.01;
		 marker_array.markers[0].scale.z = 0.2;

		 // Set the color -- be sure to set alpha to something non-zero!
		 marker_array.markers[0].color.r = 1.0f;
		 marker_array.markers[0].color.g = 0.0f;
		 marker_array.markers[0].color.b = 0.0f;
		 marker_array.markers[0].color.a = 1.0;

		 // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		 marker_array.markers[1].header.frame_id = "/camera_link";
		 marker_array.markers[1].header.stamp = ros::Time::now();

		 // Set the namespace and id for this marker.  This serves to create a unique ID
		 // Any marker sent with the same namespace and id will overwrite the old one
		 marker_array.markers[1].ns = "marker2";
		 marker_array.markers[1].id = 1;

		 // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		 // marker.type = visualization_msgs::Marker::ARROW;
		 marker_array.markers[1].type = visualization_msgs::Marker::ARROW;

		 // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		 marker_array.markers[1].action = visualization_msgs::Marker::ADD;

		 // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		 marker_array.markers[1].points.push_back(p);
		 p2.x = p.x + points[6];
		 p2.y = p.y + points[7];
		 p2.z = p.z + points[8];
		 marker_array.markers[1].points.push_back(p2);

		 // Set the scale of the marker -- 1x1x1 here means 1m on a side
		 marker_array.markers[1].scale.x = 0.01;
		 marker_array.markers[1].scale.y = 0.01;
		 marker_array.markers[1].scale.z = 0.2;

		 // Set the color -- be sure to set alpha to something non-zero!
		 marker_array.markers[1].color.r = 0.0f;
		 marker_array.markers[1].color.g = 1.0f;
		 marker_array.markers[1].color.b = 0.0f;
		 marker_array.markers[1].color.a = 1.0;


		 // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		 marker_array.markers[2].header.frame_id = "/camera_link";
		 marker_array.markers[2].header.stamp = ros::Time::now();

		 // Set the namespace and id for this marker.  This serves to create a unique ID
		 // Any marker sent with the same namespace and id will overwrite the old one
		 marker_array.markers[2].ns = "marker3";
		 marker_array.markers[2].id = 2;

		 // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		 // marker.type = visualization_msgs::Marker::ARROW;
		 marker_array.markers[2].type = visualization_msgs::Marker::ARROW;

		 // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		 marker_array.markers[2].action = visualization_msgs::Marker::ADD;

		 // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		 marker_array.markers[2].points.push_back(p);
		 p3.x = p.x + points[9];
		 p3.y = p.y + points[10];
		 p3.z = p.z + points[11];
		 marker_array.markers[2].points.push_back(p3);

		 // Set the scale of the marker -- 1x1x1 here means 1m on a side
		 marker_array.markers[2].scale.x = 0.01;
		 marker_array.markers[2].scale.y = 0.01;
		 marker_array.markers[2].scale.z = 0.2;

		 // Set the color -- be sure to set alpha to something non-zero!
		 marker_array.markers[2].color.r = 0.0f;
		 marker_array.markers[2].color.g = 0.0f;
		 marker_array.markers[2].color.b = 1.0f;
		 marker_array.markers[2].color.a = 1.0;

		 marker_array.markers[0].lifetime = ros::Duration();
		 marker_array.markers[1].lifetime = ros::Duration();
		 marker_array.markers[2].lifetime = ros::Duration();

		m_marker_pub.publish(marker_array);

		ros::spinOnce ();
		loop.sleep();
	}
}
