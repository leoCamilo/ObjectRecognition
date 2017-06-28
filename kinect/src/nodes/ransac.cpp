#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <kinect/cloud-tools.h>

#define MAX_ITERATIONS 50
#define DISTANCE_THRESHOLD 0.02

namespace global {
    ros::Publisher pub;
    ros::Subscriber sub;

    bool inverter;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr half_cloud;
}

void ransac_callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    // clock_t start = clock();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    pcl::fromROSMsg(*cloud_msg, *global::cloud);

    int x = RAW_HALF_CLOUD_SIZE - 1;
    int i = 0;

    while (++x < RAW_CLOUD_SIZE)
        global::half_cloud->points[i++] = global::cloud->points[x];

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (MAX_ITERATIONS);
    seg.setDistanceThreshold (DISTANCE_THRESHOLD);
    seg.setInputCloud (global::half_cloud);
    seg.segment (*inliers, *coefficients);

    global::out_cloud->points.clear();
    global::out_cloud->header = global::cloud->header;

    if (global::inverter) {
        std::vector<int>::const_iterator it;

        for (it = inliers->indices.begin();  it != inliers->indices.end(); it++)
            global::out_cloud->points.push_back(global::half_cloud->points[*it]);

    } else {
        int z = 0;
        int k = -1;
        int index_size = inliers->indices.size();

        while (++k < RAW_HALF_CLOUD_SIZE)
            global::out_cloud->points.push_back(global::cloud->points[k]);

        k--;

        while (++k < RAW_CLOUD_SIZE && z < index_size)
            if (k != (inliers->indices[z] + RAW_HALF_CLOUD_SIZE))
                global::out_cloud->points.push_back(global::cloud->points[k]);
            else
                z++;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*global::out_cloud, output);
    global::pub.publish (output);

    // std::cout << "time: " << (double) (clock() - start) / CLOCKS_PER_SEC << std::endl;
}

void print_info () {
    time_t start = time(0);

    std::cout << std::endl << " P L A N E   R E M O V E   S T A R T E D" << std::endl;
    std::cout << " =======================================" << std::endl;
    std::cout << std::endl << " started at: " << ctime(&start) << std::endl;
    std::cout << std::endl << " ransac approach" << std::endl;
    std::cout << " - maximum iterations: " << MAX_ITERATIONS << std::endl;
    std::cout << " - distance threshold: " << DISTANCE_THRESHOLD << std::endl;
    std::cout << std::endl << " input channel: " << RAW_DATA_CHANNEL;
    std::cout << std::endl << " output channel: " << RANSAC_DATA_CHANNEL << std::endl;
}

int main (int argc, char** argv) {
    print_info();

    global::inverter = leo::find_arg(argc, argv, "-i");

    global::cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    global::out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    global::half_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    global::out_cloud->points.reserve(RAW_CLOUD_SIZE);
    global::half_cloud->points.resize(RAW_HALF_CLOUD_SIZE);
    global::half_cloud->width = RAW_HALF_CLOUD_SIZE;
    global::half_cloud->height = 1;
    global::half_cloud->is_dense = true;

    ros::init (argc, argv, "kinect_ransac");
    ros::NodeHandle n;

    global::sub = n.subscribe<sensor_msgs::PointCloud2> (RAW_DATA_CHANNEL, 1, ransac_callback);
    global::pub = n.advertise<sensor_msgs::PointCloud2> (RANSAC_DATA_CHANNEL, 1);

    ros::spin ();
}
