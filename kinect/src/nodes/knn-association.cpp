#include <kinect/cloud-tools.h>

// for each cloud, get center, and assossiate to a pre list
// give a name to show association

namespace global {
    ros::Publisher pub[CLOUD_QTD];
    ros::Subscriber sub[CLOUD_QTD];

    // pcl::PointCloud<pcl::PointXYZ>::Ptr clouds[CLOUD_QTD];

    // ros::Publisher mass_pub;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr mass_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int k = 0;
    int associates_index[CLOUD_QTD] = {0};

    // leo::cloud clouds[CLOUD_QTD];
    std::vector<leo::cloud> clouds_vector;
}

void association_callback (const sensor_msgs::PointCloud2ConstPtr & input_cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud_msg, *cloud);

    // // if (k++ > 100) { mass_center_cloud->points.clear(); }

    // leo::set_nearby(clouds_vector, cloud);
    // leo::publish_clouds(clouds_vector, pub);

    // mass_center_cloud->points.push_back(leo::get_center(*cloud));
    // mass_center_cloud->width = mass_center_cloud->points.size();

    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*mass_center_cloud, output);
    // mass_pub.publish(output);

    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud, output);
    // pub[i++].publish(output);

    // --------------------------------------------------------------------------------------

    pcl::PointXYZ centroid = leo::get_center(cloud);
    pcl::PointXYZ closer_centroid;
    pcl::PointXYZ tmp_centroid;

    int closer_idx = 0;
    bool matching_flag = true;
    double closer_distance = 1000.0;
    double distance;

    for (int i = 0; i < CLOUD_QTD; ++i) {
        if (global::associates_index[i] > 100) {
            // printf("index  %d has no associated cluster for more than 100 frames\n", i);
            global::clouds_vector[i].non_initialized = true;
            global::associates_index[i] = 0;
        }

        // TODO: implement some valitation mechanism on clouds to they get in to association vector

        if (global::clouds_vector[i].non_initialized && matching_flag) {
            global::clouds_vector[i].pcl_cloud_ptr = cloud;
            global::clouds_vector[i].center_point = centroid;
            global::clouds_vector[i].non_initialized = false;

            leo::publish_cloud(global::clouds_vector[i].pcl_cloud_ptr, global::pub[i]);
            return;
        }

        tmp_centroid = leo::get_center(global::clouds_vector[i].pcl_cloud_ptr);
        distance = leo::euclidian_distance(centroid, tmp_centroid);

        // TODO: reconize new clusters and if old ones are no longer used, remove then
        // save indexs and put a counter on each one, when counter become to 0  clean that position
        // compare dimenssions too
        // improve algorithm

        // printf("function %d index %d distance %f\n", function_idx, i, distance);

        if (distance < MIN_CENTER_DISTANCE)
            if (distance < closer_distance) {   // maybe is changing wrong, ajust parametheres better
                closer_idx = i;
                closer_distance = distance;
                closer_centroid = tmp_centroid;
                matching_flag = false;
            }

        global::associates_index[i]++;
    }

    if (!matching_flag) {
        global::associates_index[closer_idx] = 0;

        // do a function, that atributes the cloud and get all info
        global::clouds_vector[closer_idx].pcl_cloud_ptr = cloud;
        global::clouds_vector[closer_idx].center_point = closer_centroid;

        leo::publish_cloud(global::clouds_vector[closer_idx].pcl_cloud_ptr, global::pub[closer_idx]);
    }
}

int main (int argc, char** argv) {
    std::cout << std::endl << " A S S O C I A T I O N   S T A R T E D" << std::endl;
    std::cout << " =====================================" << std::endl;
    std::cout << std::endl << " nearest neighbors approach" << std::endl;
    std::cout << " - distance tolerance: " << MIN_CENTER_DISTANCE << std::endl;
    std::cout << std::endl << " input channel: clustering/[n]" << std::endl;
    std::cout << std::endl << " output channels:" << std::endl;

    ros::init (argc, argv, "kinect_association");
    ros::NodeHandle n;
    std::string sub_str("clustering/0");
    std::string pub_str("association/0");

    for (int z = 0; z < CLOUD_QTD; z++) {

        leo::cloud tmp_cloud;
        tmp_cloud.non_initialized = true;
        tmp_cloud.pcl_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_cloud.pcl_cloud_ptr->height = 1;
        tmp_cloud.pcl_cloud_ptr->is_dense = true;
        tmp_cloud.pcl_cloud_ptr->header.frame_id = FRAME_ID;

        global::clouds_vector.push_back(tmp_cloud);

        sub_str[11] = pub_str[12] = z + 48;
        global::sub[z] = n.subscribe <sensor_msgs::PointCloud2> (sub_str, 1, association_callback);
        global::pub[z] = n.advertise <sensor_msgs::PointCloud2> (pub_str, 1);
    }

    // mass_center_cloud->height = 1;
    // mass_center_cloud->is_dense = true;
    // mass_center_cloud->header.frame_id = "camera_depth_optical_frame";
    // mass_pub = n.advertise <sensor_msgs::PointCloud2> ("mass_center_pub", 1);

    ros::spin ();
}
