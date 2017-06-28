#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

int main (int argc, char** argv) {
    pcl::PCDReader reader;

    if (argc == 2) {
        pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::visualization::PCLVisualizer viewer ("Cloud Viewer");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color (normal_cloud, 0, 255, 0);

        reader.read (argv[1], *normal_cloud);

        viewer.setBackgroundColor (0, 0, 0);
        viewer.addPointCloud<pcl::PointNormal> (normal_cloud, single_color, "cloud");
        viewer.addPointCloudNormals<pcl::PointNormal> (normal_cloud, 10, 0.05, "normals");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        viewer.initCameraParameters ();

        while (!viewer.wasStopped ()) { viewer.spinOnce (100); }
        return 0;
    }

    if (argc == 3) {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
        pcl::visualization::PCLPlotter plotter;
        std::vector<std::pair<double, double> > data;

        reader.read (argv[1], *vfhs);

        for (int i = 0; i < 308; i++)
            data.push_back(std::pair<double, double>(i, vfhs->points[0].histogram[i]));

        plotter.addPlotData (data, "vfh signature");
        plotter.setWindowSize (600, 300);
        plotter.setTitle ("VFH Signature");
        plotter.setXTitle("Nuber of histogram bins");
        plotter.setYTitle("Percentage of points falling in each bin");
        plotter.plot();

        return 0;
    }

    printf("\nfile name missing\n");
    return -1;
}
