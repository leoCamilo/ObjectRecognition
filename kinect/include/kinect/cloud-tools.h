#ifndef __CLOUD_TOOLS_LIB_H
#define __CLOUD_TOOLS_LIB_H

#include <string>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#define VOXEL_X_SIZE 0.05
#define VOXEL_Y_SIZE 0.05
#define VOXEL_Z_SIZE 0.05

#define CLOUD_QTD 10
#define FLOAT_SIZE 4

#define MIN_CENTER_DISTANCE 0.5
#define CLUSTER_TOLERANCE 0.1	    // in cm // see which option is better, bigger ou smaller cluster tolerance, because the noise
#define MIN_CLUSTER_SIZE 50
#define MAX_CLUSTER_SIZE 1000
#define VFH_SIZE 308

#define RAW_CLOUD_SIZE 307200
#define RAW_HALF_CLOUD_SIZE 153600

#define FRAME_ID "camera_depth_optical_frame"

#define RAW_DATA_CHANNEL "/camera/depth/points"
#define FILTER_DATA_CHANNEL "/filtered/points"
#define RANSAC_DATA_CHANNEL "/ransac/points"

namespace leo {

    typedef struct cloud cloud;
    typedef struct variables vars;

    struct cloud {
	int id;
	bool non_initialized;

	double x_size;
	double y_size;
	double z_size;

	pcl::PointXYZ min_point;
	pcl::PointXYZ max_point;
	pcl::PointXYZ center_point;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    };

    struct variables {
	int pcd_type;
	int cloud_qtd = 10;
	int min_cluster_size = 30;
	int max_cluster_size = 25000;

	std::string str;
	bool classify = false;
	bool get_model = false;
	bool pub_photo = false;
	bool take_photo = false;
	bool take_photos = false;
	bool align_clouds = false;

	float voxel_x_size = 0.05;
	float voxel_y_size = 0.05;
	float voxel_z_size = 0.05;
	float min_center_distance = 0.5;
	float cluster_tolerance = 0.15;
    };

    bool find_arg(int argc, char** argv, char* paramether);

    double euclidian_distance (pcl::PointXYZ& pt1, pcl::PointXYZ& pt2);

    pcl::PointXYZ get_center (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    cloud get_info (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

    void set_nearby (std::vector<cloud>& cloud_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud);

    void publish_clouds (std::vector<cloud>& cloud_vector, ros::Publisher * pub);

    void publish_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher pub);

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr voxel_filter (const pcl::PointCloud<pcl::PointNormal>::Ptr unfiltered_cloud);

    void estimate_normals (const pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, double radius);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr get_vfh_histogram (pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int iterations, double threshold);

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> euclidian_clustering (const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, double tolerance, int min, int max);

    double ChiSquareDistance (float model[VFH_SIZE], float histogram[VFH_SIZE]);

    visualization_msgs::Marker get_draw_box (leo::cloud& current_cloud);
}

#endif
