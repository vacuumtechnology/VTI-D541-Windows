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

#include <vtkRenderWindow.h>
#include <iostream>
#include <thread>

float CalculateResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud) {
    double resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud(sceneCloud);

    std::cout << "Calculating Resolution" << std::endl;

    for (std::size_t i = 0; i < sceneCloud->size(); ++i)
    {
        if (!std::isfinite((*sceneCloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            resolution += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        resolution /= n_points;
    }

    return resolution;
}

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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPCL;
        pointCloudPCL.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p2;
        p2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, *pointCloudPCL);
        std::cout << "Loaded " << pointCloudPCL->width * pointCloudPCL->height << " points" << std::endl;
        std::cout << "width: " << pointCloudPCL->width << " points" << std::endl;
        std::cout << "resolution: " << CalculateResolution(pointCloudPCL) << std::endl;

       

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointCloudPCL, *p2, indices);
        std::cout << "size2: " << p2->size() << " points" << std::endl;

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");
        viewer.setSize(900, 1000);
        viewer.setBackgroundColor(.3, .3, .3);
        viewer.setCameraPosition(0, 0, -40, 0, -1, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloudPCL);
        viewer.resetCamera();


        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
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
