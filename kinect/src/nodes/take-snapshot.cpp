#include <kinect/cloud-tools.h>
#include <chrono>

namespace global {
    int count = 4;
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

void snapshot (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    pcl::PointCloud<pcl::PointNormal>::Ptr raw_normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh;
    pcl::fromROSMsg(*cloud_msg, *raw_normal_cloud);
    pcl::PCDWriter writer;

    leo::estimate_normals (raw_normal_cloud, 0.1);  // 10 cm
    vfh = leo::get_vfh_histogram(raw_normal_cloud);

    std::string path("/home/leo/" + currentDateTime());

    writer.write(path + "-" + std::to_string(global::count), *raw_normal_cloud, false);
    writer.write(path + "-" + std::to_string(global::count) + "-vfh", *vfh, false);

    ROS_INFO_STREAM("modelig ended with success");

    if (!global::count--)
        exit(0);
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "kinect_snapshot");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe <sensor_msgs::PointCloud2> (argv[1], 1, snapshot);

    ros::spin ();
}
