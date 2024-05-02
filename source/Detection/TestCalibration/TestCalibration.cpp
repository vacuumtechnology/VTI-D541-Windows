/*
Read point cloud from PCL file and visualize it.

The PCD file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <thread>
#include <fstream>

typedef pcl::PointXYZRGB PointType;

// Select Point on cloud using shift + Left Click
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event)
{

    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }

    event.getPoint(x, y, z);

    std::cout << "Point ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

int main(int argc, char** argv)
{
    std::string pointCloudFile, calFilename;
    Eigen::Matrix4f transform;

    try
    {
        if (argc > 2) {
            pointCloudFile = argv[1];
            calFilename = argv[2];
        } else {
            cout << "bad args" << endl;
            exit(0);
        }

        std::ifstream calFile(calFilename);
        if (calFile.is_open())
        {
            float val;
            cout << "calibration transform" << endl;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    if (calFile >> val) {
                        transform(i, j) = val;
                        cout << val << endl;
                    } else {
                        cerr << "error reading robot calibration" << endl;
                        exit(0);
                    }
                }
            }
        }


        std::cout << "Reading PCD point cloud from file: " << pointCloudFile << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPCL;
        pointCloudPCL.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p2;
        p2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, *pointCloudPCL);
        std::cout << "Loaded " << pointCloudPCL->width * pointCloudPCL->height << " points" << std::endl;
        std::cout << "width: " << pointCloudPCL->width << " points" << std::endl;



        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointCloudPCL, *p2, indices);
        std::cout << "size2: " << p2->size() << " points" << std::endl;

        pcl::transformPointCloud(*pointCloudPCL, *pointCloudPCL, transform);

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");
        viewer.setSize(900, 1000);
        viewer.setBackgroundColor(.3, .3, .3);
        viewer.setCameraPosition(0, 0, -40, 0, -1, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloudPCL);
        viewer.resetCamera();
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud");
        viewer.registerPointPickingCallback(pointPickingEventOccurred);


        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    } catch (const std::exception& e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
