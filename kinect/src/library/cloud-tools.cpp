#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kinect/cloud-tools.h>

namespace leo {

    int _id = 0;
    int associates_index[CLOUD_QTD] = {0};

    bool find_arg(int argc, char** argv, char* paramether) {
        std::string arg(paramether);

        while (argc--)
            if (!arg.compare(argv[argc]))
                return true;

        return false;
    }

    double euclidian_distance (pcl::PointXYZ& pt1, pcl::PointXYZ& pt2) {
        double delta_x = pt1.x - pt2.x;
        double delta_y = pt1.y - pt2.y;
        double delta_z = pt1.z - pt2.z;

        return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    pcl::PointXYZ get_center (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointXYZ min_point, max_point, centroid;
        pcl::getMinMax3D(*cloud, min_point, max_point);

        centroid.x = (max_point.x + min_point.x) / 2.0;
        centroid.y = (max_point.y + min_point.y) / 2.0;
        centroid.z = (max_point.z + min_point.z) / 2.0;

        return centroid;
    }

    cloud get_info (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
        cloud tmp_cloud;

        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(*input_cloud, min_point, max_point);

        tmp_cloud.min_point = min_point;
        tmp_cloud.max_point = max_point;

        tmp_cloud.x_size = max_point.x - min_point.x;
        tmp_cloud.y_size = max_point.y - min_point.y;
        tmp_cloud.z_size = max_point.z - min_point.z;

        tmp_cloud.center_point.x = (max_point.x + min_point.x) / 2.0;
        tmp_cloud.center_point.y = (max_point.y + min_point.y) / 2.0;
        tmp_cloud.center_point.z = (max_point.z + min_point.z) / 2.0;

        return tmp_cloud;
    }

    void set_nearby (std::vector<cloud> & cloud_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud) {
        // std::vector<cloud>::iterator it;

        pcl::PointXYZ centroid = get_center(current_cloud);
        pcl::PointXYZ closer_centroid;
        pcl::PointXYZ tmp_centroid;

        bool matching_flag = true;
        int closer_idx = 0;
        double closer_distance = 1000.0;
        double distance;

        for (int i = 0; i < CLOUD_QTD; ++i) {
            if (associates_index[i] > 20) {
                cloud_vector[i].non_initialized = true;
                associates_index[i] = 0;
            }

            if (cloud_vector[i].non_initialized && matching_flag) {
                cloud_vector[i].pcl_cloud_ptr = current_cloud;
                cloud_vector[i].center_point = centroid;
                cloud_vector[i].id = _id++;
                cloud_vector[i].non_initialized = false;
                return;
            }

            tmp_centroid = get_center(cloud_vector[i].pcl_cloud_ptr);
            distance = euclidian_distance(centroid, tmp_centroid);

            // TODO: reconize new clusters and if old ones are no longer used, remove then
            // save indexs and put a counter on each one, when counter become to 0  clean that position
            // compare dimenssions too
            // improve algorithm

            if (distance < MIN_CENTER_DISTANCE)
                if (distance < closer_distance) {
                    closer_idx = i;
                    closer_distance = distance;
                    closer_centroid = tmp_centroid;
                    matching_flag = false;
                }

            associates_index[i]++;
        }

        associates_index[closer_idx] = 0;
        cloud_vector[closer_idx].pcl_cloud_ptr = current_cloud;
        cloud_vector[closer_idx].center_point = closer_centroid;
    }

    void publish_clouds (std::vector<cloud> & cloud_vector, ros::Publisher * pub) {
        sensor_msgs::PointCloud2 output;

        for (int i = 0; i < CLOUD_QTD; ++i) {
            pcl::toROSMsg(*cloud_vector[i].pcl_cloud_ptr, output);
            pub[i].publish(output);
        }
    }

    void publish_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher pub) {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        pub.publish(output);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setInputCloud(unfiltered_cloud);
        grid.setLeafSize (VOXEL_X_SIZE, VOXEL_Y_SIZE, VOXEL_Z_SIZE);
        grid.filter(*out_cloud);
        return out_cloud;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr voxel_filter (const pcl::PointCloud<pcl::PointNormal>::Ptr unfiltered_cloud) {
        pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::VoxelGrid<pcl::PointNormal> grid;

        grid.setInputCloud(unfiltered_cloud);
        grid.setLeafSize (VOXEL_X_SIZE, VOXEL_Y_SIZE, VOXEL_Z_SIZE);
        grid.filter(*out_cloud);
        grid.setDownsampleAllData(true);

        return out_cloud;
    }

    void estimate_normals (const pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, double radius) {
        pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());

        ne.setInputCloud (normal_cloud);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (radius);
        ne.compute (*normal_cloud);
    }

    pcl::PointCloud<pcl::VFHSignature308>::Ptr get_vfh_histogram (pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
        pcl::VFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> vfh;

        vfh.setInputCloud (cloud);
        vfh.setInputNormals (cloud);
        vfh.setSearchMethod (tree);
        vfh.compute (*vfhs);

        return vfhs;
    }

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> euclidian_clustering (const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, double tolerance, int min, int max) {
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> out;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);

        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointNormal> cluster;

        cluster.setClusterTolerance(tolerance);
        cluster.setMinClusterSize(min);
        cluster.setMaxClusterSize(max);
        cluster.setSearchMethod(tree);
        cluster.setInputCloud(cloud);
        cluster.extract(cluster_indices);

        // int i = 0;

        if (cluster_indices.size() > 0) {
            std::vector<pcl::PointIndices>::const_iterator it;

            for (it = cluster_indices.begin();  it != cluster_indices.end(); ++it) {
                std::vector<int>::const_iterator pit;
                cloud_cluster = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);

                for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
                    cloud_cluster->points.push_back(cloud->points[*pit]);

                out.push_back(cloud_cluster);

                // cloud_cluster->height = 1;
                // cloud_cluster->is_dense = true;
                // cloud_cluster->width = cloud_cluster->points.size();
                // cloud_cluster->header.frame_id = FRAME_ID;

                // create_file = name_file + std::to_string(i++);
                // writer.write(create_file.c_str(), *cloud_cluster, false);
            }
        }

        return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int iterations, double threshold) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr half_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        const int cloud_size = cloud->points.size();
        const int half_size = cloud_size / 2;

        int x = cloud_size - half_size - 1;
        int i = 0;

        half_cloud->points.resize(half_size);

        while (++x < cloud_size)
            half_cloud->points[i++] = cloud->points[x];

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (iterations);
        seg.setDistanceThreshold (threshold);
        seg.setInputCloud (half_cloud);
        seg.segment (*inliers, *coefficients);

        int z = 0;
        int k = -1;
        int index_size = inliers->indices.size();

        out_cloud->points.resize(half_size);

        while (++k < half_size)
            out_cloud->points[k] = cloud->points[k];

        k--;

        while (++k < cloud_size && z < index_size)
            if (k != (inliers->indices[z] + half_size))
                out_cloud->points.push_back(cloud->points[k]);
            else
                z++;

        return out_cloud;
    }

    double ChiSquareDistance (float model[VFH_SIZE], float histogram[VFH_SIZE]) {
        int i = VFH_SIZE;
        double result = 0;

        while (--i >= 0)
            result += !model[i] && !histogram[i] ? 0 : ((model[i] - histogram[i]) * (model[i] - histogram[i])) / (model[i] + histogram[i]);

        return result * 0.5;
    }

    visualization_msgs::Marker get_draw_box (leo::cloud& current_cloud) {
        visualization_msgs::Marker box_marker;
        geometry_msgs::Point p1, p2, p3;

        p1.x = p2.x = p3.x = current_cloud.min_point.x;
        p1.y = p2.y = p3.y = current_cloud.min_point.y;
        p2.z = p3.z = current_cloud.max_point.z;
        p1.z = current_cloud.min_point.z;

        // ---- line 1
        box_marker.points.push_back(p1);
        box_marker.points.push_back(p2);

        // ---- line 2
        box_marker.points.push_back(p1);
        p1.x = current_cloud.max_point.x;
        p2.x = current_cloud.max_point.x;
        box_marker.points.push_back(p1);

        // ---- line 3
        box_marker.points.push_back(p3);
        p3.x = current_cloud.max_point.x;
        box_marker.points.push_back(p3);

        // ---- line 4
        box_marker.points.push_back(p1);
        box_marker.points.push_back(p2);

        // ---- line 5
        box_marker.points.push_back(p1);
        p1.y = current_cloud.max_point.y;
        p2.y = current_cloud.max_point.y;
        box_marker.points.push_back(p1);

        // ---- line 6
        box_marker.points.push_back(p3);
        p3.y = current_cloud.max_point.y;
        box_marker.points.push_back(p3);

        // ---- line 7
        box_marker.points.push_back(p1);
        box_marker.points.push_back(p2);

        // ---- line 8
        box_marker.points.push_back(p1);
        p1.x = current_cloud.min_point.x;
        p2.x = current_cloud.min_point.x;
        box_marker.points.push_back(p1);

        // ---- line 9
        box_marker.points.push_back(p3);
        p3.x = current_cloud.min_point.x;
        box_marker.points.push_back(p3);

        // ---- line 10
        box_marker.points.push_back(p1);
        box_marker.points.push_back(p2);

        // ---- line 11
        box_marker.points.push_back(p1);
        p1.y = current_cloud.min_point.y;
        p2.y = current_cloud.min_point.y;
        box_marker.points.push_back(p1);

        // ---- line 12
        box_marker.points.push_back(p3);
        p3.y = current_cloud.min_point.y;
        box_marker.points.push_back(p3);

        box_marker.id = 0;
        box_marker.color.r = 1.0f;
        box_marker.color.g = 0.0f;
        box_marker.scale.x = 0.01;
        box_marker.type = visualization_msgs::Marker::LINE_LIST;
        box_marker.header.frame_id = FRAME_ID;
        box_marker.ns = "element_box";
        box_marker.color.a = 1.0;
        box_marker.color.b = 0.0f;
        box_marker.action = visualization_msgs::Marker::ADD;

        return box_marker;
    }
}
