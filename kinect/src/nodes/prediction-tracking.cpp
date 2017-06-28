#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <kinect/cloud-tools.h>
#include <visualization_msgs/Marker.h>

#define OUT_DATA_CHANNEL "/marker/tracker"
#define OSCILATION_BOUND 0.01

namespace global {
    ros::Publisher pub;
    ros::Subscriber sub;

    visualization_msgs::Marker box_marker;
    visualization_msgs::Marker txt_marker;

    pcl::PointXYZ last_point;
    pcl::PointXYZ current_point;
}

void print_info (char* input_channel) {
    std::cout << std::endl << " T R A C K E R" << std::endl;
    std::cout << " =============" << std::endl;
    std::cout << std::endl << " traking channel: " << input_channel << std::endl;
}

void draw_box (leo::cloud& current_cloud) {
    geometry_msgs::Point p1, p2, p3;
    global::box_marker.points.clear();

    p1.x = p2.x = p3.x = current_cloud.min_point.x;
    p1.y = p2.y = p3.y = current_cloud.min_point.y;
    p2.z = p3.z = current_cloud.max_point.z;
    p1.z = current_cloud.min_point.z;

    // ---- line 1
    global::box_marker.points.push_back(p1);
    global::box_marker.points.push_back(p2);

    // ---- line 2
    global::box_marker.points.push_back(p1);
    p1.x = current_cloud.max_point.x;
    p2.x = current_cloud.max_point.x;
    global::box_marker.points.push_back(p1);

    // ---- line 3
    global::box_marker.points.push_back(p3);
    p3.x = current_cloud.max_point.x;
    global::box_marker.points.push_back(p3);

    // ---- line 4
    global::box_marker.points.push_back(p1);
    global::box_marker.points.push_back(p2);

    // ---- line 5
    global::box_marker.points.push_back(p1);
    p1.y = current_cloud.max_point.y;
    p2.y = current_cloud.max_point.y;
    global::box_marker.points.push_back(p1);

    // ---- line 6
    global::box_marker.points.push_back(p3);
    p3.y = current_cloud.max_point.y;
    global::box_marker.points.push_back(p3);

    // ---- line 7
    global::box_marker.points.push_back(p1);
    global::box_marker.points.push_back(p2);

    // ---- line 8
    global::box_marker.points.push_back(p1);
    p1.x = current_cloud.min_point.x;
    p2.x = current_cloud.min_point.x;
    global::box_marker.points.push_back(p1);

    // ---- line 9
    global::box_marker.points.push_back(p3);
    p3.x = current_cloud.min_point.x;
    global::box_marker.points.push_back(p3);

    // ---- line 10
    global::box_marker.points.push_back(p1);
    global::box_marker.points.push_back(p2);

    // ---- line 11
    global::box_marker.points.push_back(p1);
    p1.y = current_cloud.min_point.y;
    p2.y = current_cloud.min_point.y;
    global::box_marker.points.push_back(p1);

    // ---- line 12
    global::box_marker.points.push_back(p3);
    p3.y = current_cloud.min_point.y;
    global::box_marker.points.push_back(p3);
}

void tracking_callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    leo::cloud current_cloud = leo::get_info(cloud);
    global::current_point = current_cloud.center_point;
    draw_box(current_cloud);

    std::string txt_marker_str;
    float time_distance = leo::euclidian_distance(global::current_point, global::last_point);

    if (time_distance > OSCILATION_BOUND) {
        txt_marker_str = std::to_string(time_distance) + "m/s";
    } else {
        txt_marker_str = "0.000 m/s";
    }

    global::txt_marker.text = txt_marker_str.c_str();
    global::txt_marker.pose.position.x = current_cloud.max_point.x * 2;
    global::txt_marker.pose.position.y = current_cloud.min_point.y;
    global::txt_marker.pose.position.z = current_cloud.min_point.z;

    global::pub.publish(global::txt_marker);
    global::pub.publish(global::box_marker);

    global::last_point = global::current_point;
}

void initialize_markers () {
    global::box_marker.id = 0;
    global::box_marker.color.r = 1.0f;
    global::box_marker.color.g = 0.0f;
    global::box_marker.scale.x = 0.01;
    global::box_marker.type = visualization_msgs::Marker::LINE_LIST;

    global::txt_marker.id = 1;
    global::txt_marker.scale.z = 0.1;
    global::txt_marker.color.r = 0.0f;
    global::txt_marker.color.g = 1.0f;
    global::txt_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    global::txt_marker.ns = global::box_marker.ns = "element_box";
    global::txt_marker.color.a = global::box_marker.color.a = 1.0;
    global::txt_marker.color.b = global::box_marker.color.b = 0.0f;
    global::txt_marker.header.frame_id = global::box_marker.header.frame_id = FRAME_ID;
    global::txt_marker.action = global::box_marker.action = visualization_msgs::Marker::ADD;
}

int main (int argc, char** argv) {

    print_info(argv[1]);
    initialize_markers();

    ros::init (argc, argv, "kinect_tracking");
    ros::NodeHandle n;

    global::sub = n.subscribe <sensor_msgs::PointCloud2> (argv[1], 1, tracking_callback);
    global::pub = n.advertise <visualization_msgs::Marker> (OUT_DATA_CHANNEL, 1);

    ros::spin ();
}
