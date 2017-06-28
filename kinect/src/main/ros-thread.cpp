#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

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

// #include <iostream>
// #include <pcl/visualization/cloud_viewer.h>

#include <kinect/cloud-tools.h>
#include <mutex>

#define OUT_DATA_CHANNEL "/kinect_config/points"
#define OUT_NORMALS_CHANNEL "/kinect_config/normals"

namespace leo {
    vars* var_ptr;
    ros::Publisher pub;
    ros::Publisher pub_marker;
    ros::Subscriber sub;
    std::mutex* mtx_ptr;
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::string archive_name;
    sensor_msgs::PointCloud2 output;
    visualization_msgs::MarkerArray cloud_normals;

    // ------------------- tmp
    ros::Publisher pub1;
    ros::Publisher pub2;
    // ------------------- tmp

    std::vector<std::string> split(const std::string& str, char delim) {
        std::vector<std::string> elements;
        std::stringstream ss;
        std::string item;
        ss.str(str);

        while (getline(ss, item, delim))
            elements.push_back(item);

        return elements;
    }

    void callback (const sensor_msgs::PointCloud2ConstPtr& ros_cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

        std::vector<int> idxs;
        pcl::fromROSMsg(*ros_cloud_msg, *raw_cloud);
        pcl::removeNaNFromPointCloud(*raw_cloud, *clean_cloud, idxs);
        filtered_cloud = voxel_filter(raw_cloud);

        mtx_ptr->lock();

        if (var_ptr->take_photos) {
            std::string name_file = var_ptr->str + ".";
            var_ptr->take_photos = false;
            mtx_ptr->unlock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster;
            std::string create_file;
            int i = 0;

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(filtered_cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;

            cluster.setClusterTolerance(CLUSTER_TOLERANCE);
            cluster.setMinClusterSize(MIN_CLUSTER_SIZE);
            cluster.setMaxClusterSize(MAX_CLUSTER_SIZE);
            cluster.setSearchMethod(tree);
            cluster.setInputCloud(filtered_cloud);
            cluster.extract(cluster_indices);

            if (cluster_indices.size() > 0) {
                std::vector<pcl::PointIndices>::const_iterator it;

                for (it = cluster_indices.begin();  it != cluster_indices.end(); ++it) {
                    std::vector<int>::const_iterator pit;
                    cloud_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

                    for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
                        cloud_cluster->points.push_back(filtered_cloud->points[*pit]);

                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;
                    cloud_cluster->width = cloud_cluster->points.size();
                    cloud_cluster->header.frame_id = FRAME_ID;

                    create_file = name_file + std::to_string(i++);
                    writer.write(create_file.c_str(), *cloud_cluster, false);
                }
            }

            return;
        }

        if (var_ptr->take_photo) {
            std::string name_file = var_ptr->str;
            std::string vfh_file = var_ptr->str + "_vfh";
            var_ptr->take_photo = false;
            mtx_ptr->unlock();

            /*
            pcl::PointCloud<pcl::PointXYZ>::Ptr floor_removed_cloud;
            pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh;

            floor_removed_cloud = ransac_filter (clean_cloud, 100, 0.01);   // improve removing this copy process
            pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*floor_removed_cloud, *normal_cloud);

            std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds = euclidian_clustering (normal_cloud, 0.15, 100, 25000);
            std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>::const_iterator it;

            int x = 0;

            for (it = clouds.begin();  it != clouds.end(); ++it) {
                estimate_normals (*it, 0.03);  // 3cm
                vfh = get_vfh_histogram(*it);
                writer.write((name_file + std::to_string(x)).c_str(), **it, false);
                writer.write((vfh_file + std::to_string(x++)).c_str(), *vfh, false);
            }
            */

            pcl::PointCloud<pcl::PointNormal>::Ptr raw_normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr clean_normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud;

            pcl::fromROSMsg(*ros_cloud_msg, *raw_normal_cloud);
            // pcl::removeNaNFromPointCloud(*raw_normal_cloud, *clean_normal_cloud, idxs);

            // estimate_normals (clean_normal_cloud, 0.03);  // 3cm
            // normal_cloud = voxel_filter(clean_normal_cloud);
            normal_cloud = voxel_filter(raw_normal_cloud);
            estimate_normals (normal_cloud, 0.1);  // 10 cm

            writer.write((name_file + std::to_string(1)).c_str(), *normal_cloud, false);
            writer.write((name_file + std::to_string(2)).c_str(), *raw_normal_cloud, false);

            ROS_INFO_STREAM("modelig ended with success");
            return;
        }

        if (var_ptr->pub_photo) {
            std::string file_name = var_ptr->str;
            int pcd_type = var_ptr->pcd_type;
            int i;
            var_ptr->pub_photo = false;
            mtx_ptr->unlock();

            if (!file_name.compare(archive_name) || !archive_name.compare("")) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);

                switch (pcd_type) {
                    case 0: reader.read (file_name.c_str(), *xyz_cloud); pcl::toROSMsg(*xyz_cloud, output); break;
                    case 1: reader.read (file_name.c_str(), *xyz_cloud); pcl::toROSMsg(*xyz_cloud, output); break;

                    case 2:
                        reader.read (file_name.c_str(), *normal_cloud);
                        i = normal_cloud->width;
                        cloud_normals.markers.resize(i);

                        while (--i >= 0) {
                            cloud_normals.markers[i].header.frame_id = FRAME_ID;
                            cloud_normals.markers[i].header.stamp = ros::Time();
                            cloud_normals.markers[i].ns = "viewer_cloud_normals";
                            cloud_normals.markers[i].id = i;
                            cloud_normals.markers[i].type = visualization_msgs::Marker::ARROW;
                            cloud_normals.markers[i].action = visualization_msgs::Marker::ADD;
                            cloud_normals.markers[i].pose.position.x = normal_cloud->points[i].x;
                            cloud_normals.markers[i].pose.position.y = normal_cloud->points[i].y;
                            cloud_normals.markers[i].pose.position.z = normal_cloud->points[i].z;
                            cloud_normals.markers[i].pose.orientation.x = normal_cloud->points[i].normal_x;
                            cloud_normals.markers[i].pose.orientation.y = normal_cloud->points[i].normal_y;
                            cloud_normals.markers[i].pose.orientation.z = normal_cloud->points[i].normal_z;
                            cloud_normals.markers[i].pose.orientation.w = normal_cloud->points[i].curvature;
                            cloud_normals.markers[i].scale.x = 0.01;
                            cloud_normals.markers[i].scale.y = 0.001;
                            cloud_normals.markers[i].scale.z = 0.001;
                            cloud_normals.markers[i].color.a = 1.0;
                            cloud_normals.markers[i].color.r = 1.0;
                            cloud_normals.markers[i].color.g = 1.0;
                            cloud_normals.markers[i].color.b = 1.0;
                        }

                        pub_marker.publish(cloud_normals);
                        pcl::toROSMsg(*normal_cloud, output);
                        break;

                    case 3: reader.read (file_name.c_str(), *normal_cloud); pcl::toROSMsg(*normal_cloud, output); break;

                    default: return;
                }

                output.header.frame_id = FRAME_ID;
            }

            pub.publish (output);
            return;
        }

        if (var_ptr->align_clouds) {
            var_ptr->align_clouds = false;
            std::string files_name = var_ptr->str;
            mtx_ptr->unlock();

            std::vector<std::string> files_array = split(files_name, '|');
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);

            reader.read (files_array[0].c_str(), *cloud_1);
            reader.read (files_array[1].c_str(), *cloud_2);

            // cloud_in->width    = 5;
            // cloud_in->height   = 1;
            // cloud_in->is_dense = false;
            // cloud_in->points.resize (cloud_in->width * cloud_in->height);

            // for (size_t i = 0; i < cloud_in->points.size (); ++i) {
            //     cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
            //     cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            //     cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
            // }

            // std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;

            // for (size_t i = 0; i < cloud_in->points.size (); ++i)
            //     std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;

            // *cloud_out = *cloud_in;
            // std::cout << "size:" << cloud_out->points.size() << std::endl;

            // for (size_t i = 0; i < cloud_in->points.size (); ++i) {
            //     cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
            //     // cloud_out->points[i].y = cloud_in->points[i].y + 0.2f;
            //     cloud_out->points[i].z = cloud_in->points[i].z + 1.5f;
            // }

            // std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;

            // for (size_t i = 0; i < cloud_out->points.size (); ++i)
            //     std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

            // icp.setInputCloud(cloud_in);

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(cloud_1);
            icp.setInputTarget(cloud_2);
            pcl::PointCloud<pcl::PointXYZ> cloud_out;
            icp.align(cloud_out);

            std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;

            pcl::PCDWriter writer;
            writer.write(files_array[2].c_str(), cloud_out, false);

            return;
        }

        if (var_ptr->get_model) {
            var_ptr->get_model = false;
            std::string file_str = var_ptr->str;
            mtx_ptr->unlock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            reader.read (file_str.c_str(), *cloud);

            int z = 1;
            int cloud_size = cloud->points.size();
            int histogram_size = (cloud_size * cloud_size - cloud_size) / 2;

            std::vector<float> histogram (histogram_size + 1);
            histogram[0] = histogram_size++;

            for (int i = 0; i < cloud_size; i++)
                for (int j = i + 1; j < cloud_size; j++)
                    histogram[z++] = euclidian_distance(cloud->points[i], cloud->points[j]);


            std::fstream file;

            file_str +=  ".histogram";
            file.open(file_str.c_str(), std::ios::out | std::ios::binary);

            if (!file.is_open()) {
                std::cout << "problem to open file" << std::endl;
                return;
            }

            file.write((char*) histogram.data(), histogram_size * FLOAT_SIZE);
            file.close();
            return;
        }

        if (var_ptr->classify) {
            var_ptr->classify = false;
            std::string tmp_str = var_ptr->str;
            mtx_ptr->unlock();

            std::fstream file;
            int total = 0, total2 = 0;
            float size_histogram;
            float size_tmp_histogram;
            float threshold = 0.2;

            std::vector<std::string> str_array = split(tmp_str, '|');

            // ---------------- first file
            file.open(str_array[1].c_str(), std::ios::in | std::ios::binary);
            file.read((char*) &size_histogram, FLOAT_SIZE);

            std::vector<float> histogram (size_histogram);
            file.read((char*) histogram.data(), size_histogram * FLOAT_SIZE);
            file.close();

            // ---------------- first file
            file.open(str_array[0].c_str(), std::ios::in | std::ios::binary);
            file.read((char*) &size_tmp_histogram, FLOAT_SIZE);

            std::vector<float> tmp_histogram (size_tmp_histogram);
            file.read((char*) tmp_histogram.data(), size_tmp_histogram * FLOAT_SIZE);
            file.close();

            // TODO: maybe use std::list instead vector, see if is faster

            std::cout << "cloud 1 histogram size: " << size_histogram << std::endl;
            std::cout << "cloud 2 histogram size: " << size_tmp_histogram << std::endl;
            std::cout << "max iterations: " << size_histogram * size_tmp_histogram << std::endl;

            for (size_t i = 0; i < size_histogram; i++) {
                for (size_t j = 0; j < tmp_histogram.size(); j++) {
                    total2++;

                    if ((histogram[i] + threshold) > tmp_histogram[j] && (histogram[i] - threshold) < tmp_histogram[j]) {
                        // histogram.erase(histogram.begin() + i--);
                        // tmp_histogram.erase(tmp_histogram.begin() + j--);
                        tmp_histogram.erase(tmp_histogram.begin() + j);
                        total++;
                        break;
                    }
                }
            }

            std::cout << "iterations: " << total2 << std::endl;
            std::cout << "total: " << total << std::endl << std::endl;
            return;
        }

        mtx_ptr->unlock();
    }

    void ros_main(int argc, char *argv[], vars& var, std::mutex& mtx) {
        var_ptr = &var;
        mtx_ptr = &mtx;

        ros::init (argc, argv, "kinect_config");
        ros::NodeHandle n;
        sub = n.subscribe <sensor_msgs::PointCloud2> (RAW_DATA_CHANNEL, 1, callback);
        pub = n.advertise <sensor_msgs::PointCloud2> (OUT_DATA_CHANNEL, 1);

        pub_marker = n.advertise <visualization_msgs::MarkerArray> (OUT_NORMALS_CHANNEL, 100);

        // pub1 = n.advertise <sensor_msgs::PointCloud2> ("/kinect/1", 1);
        // pub2 = n.advertise <sensor_msgs::PointCloud2> ("/kinect/2", 1);

        ros::spin ();
    }
}
