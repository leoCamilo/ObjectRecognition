#include <boost/filesystem/path.hpp>
#include <kinect/cloud-tools.h>

#define OUT_DATA_CHANNEL "/marker/tracker"

namespace global {
    ros::Publisher pub;
    ros::Subscriber sub[CLOUD_QTD];
    pcl::PCDReader reader;
    std::vector<std::pair <std::string, std::vector<float> > > vfh_matrix;
}

void print_info (char* model) {
    time_t start = time(0);

    std::cout << std::endl << " C L A S S I F I C A T I O N   S T A R T E D" << std::endl;
    std::cout << " ============================================" << std::endl;
    std::cout << std::endl << " started at: " << ctime(&start) << std::endl;
    std::cout << std::endl << " vfh (viewpoint feature histogram) approach";
    std::cout << std::endl << " model: " << model << std::endl;
}

bool load_histograms (const boost::filesystem::path & path) {
    bool success = false;

    if (boost::filesystem::is_directory(path)) {
        boost::filesystem::directory_iterator end_it;

        for (boost::filesystem::directory_iterator file_it(path); file_it != end_it; ++file_it)
            if (boost::filesystem::is_regular_file(file_it->status())) {
                std::string full_file_name = file_it->path().string();

                if (full_file_name.find("vfh") < full_file_name.size()) {
                    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
                    std::vector<float> data;

                    global::reader.read (full_file_name, *vfhs);

                    for (int i = 0; i < 308; i++)
                        data.push_back(vfhs->points[0].histogram[i]);

                    global::vfh_matrix.push_back(std::pair<std::string, std::vector<float> >(full_file_name, data));
                    success = true;
                }
            }
    }

    return success;
}

void vfh_callback (const sensor_msgs::PointCloud2ConstPtr & cloud_msg) {
    double result = 999999999;
    double tmp;
    int models_count = global::vfh_matrix.size();
    int idx;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr raw_normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh;

    pcl::fromROSMsg(*cloud_msg, *raw_cloud);
    pcl::fromROSMsg(*cloud_msg, *raw_normal_cloud);

    leo::estimate_normals (raw_normal_cloud, 0.1);  // 10 cm
    vfh = leo::get_vfh_histogram(raw_normal_cloud);

    for (int x = 0; x < models_count; ++x) {
        tmp = leo::ChiSquareDistance(vfh->points[0].histogram, global::vfh_matrix[x].second.data());

        if (tmp < result) {
            idx = x;
            result = tmp;
        }
    }

    leo::cloud current_cloud = leo::get_info(raw_cloud);

    if (result < 70) {
        visualization_msgs::Marker box_marker = leo::get_draw_box(current_cloud);
        global::pub.publish(box_marker);

        std::cout << "x: " << current_cloud.center_point.x << " y: " << current_cloud.center_point.y << " z: " << current_cloud.center_point.z;
        std::cout << " name: " <<  global::vfh_matrix[idx].first << " score: " << result << std::endl;
    }else{
        std::cout << "x: " << current_cloud.center_point.x << " y: " << current_cloud.center_point.y << " z: " << current_cloud.center_point.z;
        std::cout << " false negative - score: " << result << std::endl;
    }
}

int main (int argc, char** argv) {
    print_info(argv[1]);

    ros::init (argc, argv, "kinect_vfh");
    ros::NodeHandle n;

    if (!load_histograms(argv[1]))
        return -1;

    std::string sub_str("association/0");

    for (int z = 0; z < CLOUD_QTD; z++) {
        sub_str[12] = z + 48;
        global::sub[z] = n.subscribe <sensor_msgs::PointCloud2> (sub_str, 1, vfh_callback);
    }

    global::pub = n.advertise <visualization_msgs::Marker> (OUT_DATA_CHANNEL, 1);

    ros::spin ();
}
