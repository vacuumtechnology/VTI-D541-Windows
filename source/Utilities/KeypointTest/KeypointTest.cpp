/* \author Bastian Steder */
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/time.h> // for StopWatch
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/harris_3d.h>

int
main(int argc, char** argv)
{
    if (argc < 2)
    {
        pcl::console::print_info("Keypoints indices example application.\n");
        pcl::console::print_info("Syntax is: %s <source-pcd-file>\n", argv[0]);
        return (1);
    }

    pcl::console::print_info("Reading %s\n", argv[1]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) // load the file
    {
        pcl::console::print_error("Couldn't read file %s!\n", argv[1]);
        return (-1);
    }

    pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression(false);
    detector.setInputCloud(cloud);
    //detector.setThreshold(.1);
    detector.setNumberOfThreads(10);
    pcl::StopWatch watch;
    detector.compute(*keypoints);
    pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size(), watch.getTimeSeconds());
    pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices();
    if (!keypoints_indices->indices.empty())
    {
        pcl::io::savePCDFile("keypoints.pcd", *cloud, keypoints_indices->indices, true);
        pcl::console::print_info("Saved keypoints to keypoints.pcd\n");
    } else
        pcl::console::print_warn("Keypoints indices are empty!\n");

    auto viewer = pcl::visualization::PCLVisualizer("Viewer");
    /*int *id = (int *)viewer.getRenderWindow()->GetGenericWindowId();
    cout << "id: " << *id << endl;*/
    viewer.setBackgroundColor(.3, .3, .3);
    viewer.setCameraPosition(0, 0, -100, 0, -1, 0);
    viewer.addPointCloud<pcl::PointXYZI>(keypoints);


    std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
    std::cout << "Press q to exit the viewer application" << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}