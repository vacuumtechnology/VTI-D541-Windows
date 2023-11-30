/*
Read point cloud from PCL file and visualize it.

The PCD file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkRenderWindow.h>
#include <iostream>
#include <thread>

namespace
{
    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGBNormal>(pointCloud);

        const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pointCloud, normalsSkipped, 1, "normals");
    }

    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud);
    }

    template<typename T>
    void visualizePointCloud(const pcl::PointCloud<T> &pointCloud)
    {
        
    }
} // namespace

int main(int argc, char **argv)
{
    std::string pointCloudFile;

    try
    {
        if (argc > 1) {
            pointCloudFile = argv[1];
        } else {
            pointCloudFile = "../../pcd/Capture.pcd";
        }

        
        std::cout << "Reading PCD point cloud from file: " << pointCloudFile << std::endl;

        auto pointCloudPCL = pcl::PointCloud<pcl::PointXYZRGB>();

        pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, pointCloudPCL);
        std::cout << "Loaded " << pointCloudPCL.width * pointCloudPCL.height << " points" << std::endl;
        std::cout << "width: " << pointCloudPCL.width << " points" << std::endl;

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");
        viewer.setSize(900, 1000);
        viewer.setBackgroundColor(.3, .3, .3);
        viewer.setCameraPosition(0, 0, -40, 0, -1, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloudPCL.makeShared());
        viewer.resetCamera();


        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
