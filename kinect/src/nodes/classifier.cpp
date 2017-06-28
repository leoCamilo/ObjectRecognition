#include <vector>
#include <fstream>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>

#include <kinect/cloud-tools.h>

#define THRESHOLD 0.1
#define OUT_DATA_CHANNEL "/marker/tracker"

ros::Publisher pub;
ros::Subscriber sub[CLOUD_QTD];

visualization_msgs::Marker box_marker;
visualization_msgs::Marker txt_marker;

std::string txt_marker_str;
std::vector<float> histogram;

float size_model;
unsigned int max_total;

pcl::PointXYZ last_point;
pcl::PointXYZ current_point;

void draw_box (leo::cloud& current_cloud) {
    geometry_msgs::Point p1, p2, p3;
    box_marker.points.clear();

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
}

void classify_callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    unsigned int total = 0;
    unsigned int cloud_size = cloud->points.size();

    if (cloud_size > 1000)  // TODO: improve filters
        return;

    // TODO: improve it, put another for inside the generated one to go throught the model histogram

    float distance;

    std::vector<float> cloud_histogram = histogram;
    std::vector<float>::iterator lower_bound_iterator;

    for (unsigned int i = 0; i < cloud_size; i++) {
        for (unsigned int j = i + 1; j < cloud_size; j++) {
            distance = leo::euclidian_distance(cloud->points[i], cloud->points[j]);

            // ------------------------ bounds search

            lower_bound_iterator = std::lower_bound(cloud_histogram.begin(), cloud_histogram.end(), distance - THRESHOLD);

            if (lower_bound_iterator == cloud_histogram.end())
                continue;

            if ((distance + THRESHOLD * 2) >= *lower_bound_iterator) {
                cloud_histogram.erase(lower_bound_iterator);
                total++;
            }

            // ------------------------ linear search
/*
            for (size_t z = 0; z < cloud_histogram.size(); z++) {
                loop_count++;

                if ((cloud_histogram[z] + THRESHOLD) > distance && (cloud_histogram[z] - THRESHOLD) < distance) {
                    cloud_histogram.erase(cloud_histogram.begin() + z);
                    total++;
                    break;
                }
            }
*/
        }
    }

    std::cout << "total: " << total << std::endl;
    std::cout << "---------------------" << std::endl;


    // if (total > 100000) {   // see a better marker
    if (total > 70000) {   // see a better marker
        leo::cloud current_cloud = leo::get_info(cloud);
        current_point = current_cloud.center_point;
        draw_box(current_cloud);

        pub.publish(txt_marker);
        pub.publish(box_marker);
    }
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "kinect_classifier");
    ros::NodeHandle n;
    std::fstream file;
    std::string sub_str("association/0");

    box_marker.type = visualization_msgs::Marker::LINE_LIST;
    box_marker.id = 0;
    box_marker.color.r = 1.0f;
    box_marker.color.g = 0.0f;
    box_marker.scale.x = 0.01;

    txt_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt_marker.text = argv[1];
    txt_marker.id = 1;
    txt_marker.color.r = 0.0f;
    txt_marker.color.g = 1.0f;
    txt_marker.scale.z = 0.1;

    txt_marker.header.frame_id = box_marker.header.frame_id = FRAME_ID;
    txt_marker.action = box_marker.action = visualization_msgs::Marker::ADD;
    txt_marker.ns = box_marker.ns = "element_box";
    txt_marker.color.b = box_marker.color.b = 0.0f;
    txt_marker.color.a = box_marker.color.a = 1.0;

    file.open(argv[1], std::ios::in | std::ios::binary);
    file.read((char*) &size_model, FLOAT_SIZE);

    histogram.resize(size_model);
    file.read((char*) histogram.data(), size_model * FLOAT_SIZE);
    file.close();

    std::sort (histogram.begin(), histogram.end());

    // sub[0] = n.subscribe <sensor_msgs::PointCloud2> ("/kinect_config/points", 1, classify_callback);

    for (int z = 0; z < CLOUD_QTD; z++) {
        sub_str[12] = z + 48;
        sub[z] = n.subscribe <sensor_msgs::PointCloud2> (sub_str, 1, classify_callback);
    }

    pub = n.advertise <visualization_msgs::Marker> (OUT_DATA_CHANNEL, 1);

    printf("\n\nclassify started\n\n");
    ros::spin ();
}
