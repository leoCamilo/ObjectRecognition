#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "noise_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("noise/cloud", 1);
	ros::Rate loop_rate(10);

	pcl::PointCloud<pcl::PointXYZ> cloud;

	for (int i = 0; i < 10000; i++)
		cloud.push_back(pcl::PointXYZ (rand(), rand(), rand()));

	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(cloud, output);

	while (ros::ok()) {
		pub.publish(output);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}