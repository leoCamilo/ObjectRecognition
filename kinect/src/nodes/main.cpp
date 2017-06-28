#include <string>
#include <vector>
#include <sstream>
// #include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <kinect/cloud-tools.h>

#define OUT_DATA_CHANNEL "/kinect_main/cloud"

// ros::Publisher pub;
ros::Publisher pub[CLOUD_QTD];
ros::Subscriber sub;

int k = 0;
int associates_index[CLOUD_QTD] = {0};
std::vector<leo::cloud> clouds_vector;

void callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg, *raw_cloud);
    filtered_cloud = leo::voxel_filter(raw_cloud);

    tree->setInputCloud(filtered_cloud);

    int limit = 0;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;

    cluster.setClusterTolerance(CLUSTER_TOLERANCE);
    cluster.setMinClusterSize(MIN_CLUSTER_SIZE);
    cluster.setMaxClusterSize(MAX_CLUSTER_SIZE);
    cluster.setSearchMethod(tree);
    cluster.setInputCloud(filtered_cloud);
    cluster.extract(cluster_indices);

    if (cluster_indices.size() > 0 && limit < CLOUD_QTD) {
        std::vector<pcl::PointIndices>::const_iterator it;

        for (it = cluster_indices.begin();  it != cluster_indices.end(); ++it) {
            std::vector<int>::const_iterator pit;
            cloud_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

            for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(filtered_cloud->points[*pit]);

            cloud_cluster->width = cloud_cluster->points.size();

            pcl::PointXYZ centroid = leo::get_center(cloud_cluster);
            pcl::PointXYZ closer_centroid;
            pcl::PointXYZ tmp_centroid;

            int closer_idx = 0;
            bool matching_flag = true;
            double closer_distance = 1000.0;
            double distance;

            for (int i = 0; i < CLOUD_QTD; ++i) {   // TODO: generate box, and don't allow box inside others
                if (associates_index[i] > 100) {
                    clouds_vector[i].non_initialized = true;
                    associates_index[i] = 0;
                }

                if (clouds_vector[i].non_initialized && matching_flag) {
                    clouds_vector[i].pcl_cloud_ptr->width = cloud_cluster->width;
                    clouds_vector[i].pcl_cloud_ptr->points = cloud_cluster->points;
                    clouds_vector[i].center_point = centroid;
                    clouds_vector[i].non_initialized = false;
                    leo::publish_cloud(clouds_vector[i].pcl_cloud_ptr, pub[i]);
                    break;
                }

                tmp_centroid = leo::get_center(clouds_vector[i].pcl_cloud_ptr);
                distance = leo::euclidian_distance(centroid, tmp_centroid);

                if (distance < MIN_CENTER_DISTANCE)
                    if (distance < closer_distance) {
                        closer_idx = i;
                        closer_distance = distance;
                        closer_centroid = tmp_centroid;
                        matching_flag = false;
                    }

                associates_index[i]++;
            }

            if (!matching_flag) {
                associates_index[closer_idx] = 0;
                clouds_vector[closer_idx].center_point = closer_centroid;
                clouds_vector[closer_idx].pcl_cloud_ptr->width = cloud_cluster->width;
                clouds_vector[closer_idx].pcl_cloud_ptr->points = cloud_cluster->points;
                leo::publish_cloud(clouds_vector[closer_idx].pcl_cloud_ptr, pub[closer_idx]);
            }
        }
    }
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "kinect_main");
    ros::NodeHandle n;
    std::string pub_str("association/0");

    for (int z = 0; z < CLOUD_QTD; z++) {

        leo::cloud tmp_cloud;
        tmp_cloud.non_initialized = true;
        tmp_cloud.pcl_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_cloud.pcl_cloud_ptr->height = 1;
        tmp_cloud.pcl_cloud_ptr->is_dense = true;
        tmp_cloud.pcl_cloud_ptr->header.frame_id = FRAME_ID;

        clouds_vector.push_back(tmp_cloud);

        pub_str[12] = z + 48;
        pub[z] = n.advertise <sensor_msgs::PointCloud2> (pub_str, 1);
    }

    sub = n.subscribe <sensor_msgs::PointCloud2> (RAW_DATA_CHANNEL, 1, callback);
    ros::spin ();
}
