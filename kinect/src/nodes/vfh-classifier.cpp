#include <boost/filesystem/path.hpp>
#include <kinect/cloud-tools.h>

namespace global {
    // ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PCDReader reader;
    std::vector<std::pair <std::string, std::vector<float> > > vfh_matrix;
}

void print_info (char* input_channel) {
    time_t start = time(0);

    std::cout << std::endl << " C L A S S I F I C A T I O N   S T A R T E D" << std::endl;
    std::cout << " ============================================" << std::endl;
    std::cout << std::endl << " started at: " << ctime(&start) << std::endl;
    std::cout << std::endl << " vfh (viewpoint feature histogram) approach" << std::endl;
    std::cout << std::endl << " input channel: " << input_channel << std::endl;
}

void classify (const boost::filesystem::path & file) {
    if (boost::filesystem::is_regular_file(file)) {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
        std::string full_file_path = file.string();

        global::reader.read (full_file_path, *vfhs);

        double result;
        int models_count = global::vfh_matrix.size();

        for (int x = 0; x < models_count; ++x) {
            result = leo::ChiSquareDistance(vfhs->points[0].histogram, global::vfh_matrix[x].second.data());
            std::cout << "model: " << global::vfh_matrix[x].first << " result: " << result << std::endl;
        }
    }
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

    pcl::PointCloud<pcl::PointNormal>::Ptr raw_normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh;
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

    std::cout << "name: " <<  global::vfh_matrix[idx].first << " score: " << result << std::endl;
}

int main (int argc, char** argv) {
    print_info(argv[1]);

    ros::init (argc, argv, "kinect_vfh");
    ros::NodeHandle n;

    if (!load_histograms(argv[1]))
        return -1;

    // classify(argv[2]);
    global::sub = n.subscribe<sensor_msgs::PointCloud2> (argv[2], 1, vfh_callback);

    ros::spin ();
}
