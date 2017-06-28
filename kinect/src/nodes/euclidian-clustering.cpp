#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>

#include <kinect/cloud-tools.h>

namespace global {
    ros::Publisher pub[CLOUD_QTD];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster[CLOUD_QTD];
}

void cluster_callback (const sensor_msgs::PointCloud2ConstPtr & input_cloud_msg) {
    // clock_t start = clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::fromROSMsg(*input_cloud_msg, *cloud);

    tree->setInputCloud(cloud);

    cluster.setClusterTolerance(CLUSTER_TOLERANCE);
    cluster.setMinClusterSize(MIN_CLUSTER_SIZE);
    cluster.setMaxClusterSize(MAX_CLUSTER_SIZE);
    cluster.setSearchMethod(tree);
    cluster.setInputCloud(cloud);
    cluster.extract(cluster_indices);

    if (cluster_indices.size() > 0) {
        std::vector<pcl::PointIndices>::const_iterator it;
        sensor_msgs::PointCloud2 output;
        int i = 0;

        for (it = cluster_indices.begin();  it != cluster_indices.end(); ++it) {
            if (i >= CLOUD_QTD)
                break;

            global::cloud_cluster[i]->points.clear();
            std::vector<int>::const_iterator idx_it;

            for (idx_it = it->indices.begin(); idx_it != it->indices.end(); idx_it++)
                global::cloud_cluster[i]->points.push_back(cloud->points[*idx_it]);

            global::cloud_cluster[i]->width = global::cloud_cluster[i]->points.size();
            pcl::toROSMsg(*global::cloud_cluster[i], output);
            global::pub[i++].publish(output);
        }
    } else
        ROS_INFO_STREAM("0 clusters extracted\n");

    // std::cout << "time: " << (double) (clock() - start) / CLOCKS_PER_SEC << std::endl;
}

void print_info () {
    time_t start = time(0);

    std::cout << std::endl << " C L U S T E R I N G   S T A R T E D" << std::endl;
    std::cout << " ===================================" << std::endl;
    std::cout << std::endl << " started at: " << ctime(&start) << std::endl;
    std::cout << std::endl << " euclidian clustering approach" << std::endl;
    std::cout << " - distance tolerance: " << CLUSTER_TOLERANCE << std::endl;
    std::cout << " - minimal cluster size: " << MIN_CLUSTER_SIZE << std::endl;
    std::cout << " - maximum cluster size: " << MAX_CLUSTER_SIZE << std::endl;
    std::cout << " - maximum clouds count: " << CLOUD_QTD << std::endl;
    std::cout << std::endl << " input channel: " << FILTER_DATA_CHANNEL << std::endl;
    std::cout << std::endl << " output channels:" << std::endl;
}

int main (int argc, char** argv) {
    print_info();

    for (int z = 0; z < CLOUD_QTD; z++) {
        std::cout << " - clustering/" << z << std::endl;

        global::cloud_cluster[z] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        global::cloud_cluster[z]->height = 1;
        global::cloud_cluster[z]->is_dense = true;
        global::cloud_cluster[z]->header.frame_id = FRAME_ID;
    }

    ros::init (argc, argv, "kinect_clustering");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> (FILTER_DATA_CHANNEL, 1, cluster_callback);
    // ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> (RANSAC_DATA_CHANNEL, 1, cluster_callback);

    std::string pub_str("clustering/0");

    for (int z = 0; z < CLOUD_QTD; z++) {
        pub_str[11] = z + 48;
        global::pub[z] = n.advertise <sensor_msgs::PointCloud2> (pub_str, 1);
    }

    ros::spin();
}
