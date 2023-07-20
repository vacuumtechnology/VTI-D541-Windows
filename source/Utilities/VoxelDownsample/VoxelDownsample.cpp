#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <windows.h> 
#include <iostream>

int
main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "./VoxelDownsample input.pcd output.pcd leafSize(optional: default 5.0)" << std::endl;
        exit(0);
    }

    float leaf;
    if (argc > 3) {
        leaf = atof(argv[3]);
    }
    else {
        leaf = 5.0f;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read(argv[1], *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf, leaf, leaf);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

    pcl::io::savePCDFileBinary(argv[2], *cloud_filtered);


    return (0);
}