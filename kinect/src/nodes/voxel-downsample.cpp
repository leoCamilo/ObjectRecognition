#include <pcl/filters/voxel_grid.h>
#include <kinect/cloud-tools.h>

#define LEAF 0.03

namespace global {
    ros::Publisher pub;
    ros::Subscriber sub;
}

void print_info (std::string in_channel, char* out_channel) {
    time_t start = time(0);

    std::cout << std::endl << " D O W N S A M P L E   S T A R T E D" << std::endl;
    std::cout << " ===================================" << std::endl;
    std::cout << std::endl << " started at: " << ctime(&start) << std::endl;
    std::cout << std::endl << " voxel grid approach" << std::endl;
    std::cout << " - voxel x size: " << LEAF << std::endl;
    std::cout << " - voxel y size: " << LEAF << std::endl;
    std::cout << " - voxel z size: " << LEAF << std::endl;
    std::cout << std::endl << " input ros channel: " << in_channel;
    std::cout << std::endl << " output ros channel: " << out_channel << std::endl;
}

void filter_callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;

    pcl_conversions::toPCL(*cloud_msg, *cloud);
    vg.setInputCloud (cloudPtr);
    vg.setLeafSize (LEAF, LEAF, LEAF);
    vg.filter (cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // std::cout << "cloud size: " << cloud->width * cloud->height << " voxel_cloud size: " << cloud_filtered.width << std::endl;
    global::pub.publish (output);
}

int main (int argc, char** argv) {
    std::string input_channel = leo::find_arg(argc, argv, "-r") ? RAW_DATA_CHANNEL : RANSAC_DATA_CHANNEL ;

    print_info(input_channel, FILTER_DATA_CHANNEL);
    ros::init (argc, argv, "kinect_downsample");
    ros::NodeHandle n;

    global::sub = n.subscribe <sensor_msgs::PointCloud2> (input_channel, 1, filter_callback);
    global::pub = n.advertise <sensor_msgs::PointCloud2> (FILTER_DATA_CHANNEL, 1);

    ros::spin ();
}
