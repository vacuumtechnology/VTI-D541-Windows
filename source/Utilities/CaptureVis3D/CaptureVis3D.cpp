/*
Capture point clouds, with settings from file, from the Zivid camera, .
*/

#include <Zivid/Zivid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

using namespace std;

typedef pcl::PointXYZRGB PointType;

class Capturer {
private:
    Zivid::Application zivid;
    Zivid::Settings settings;
    Zivid::Camera camera;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;;

public:
    Capturer::Capturer(string settingsFile) {
        std::cout << "Connecting to camera" << std::endl;
        camera = zivid.connectCamera();
        std::cout << "Connected" << std::endl;
        settings = Zivid::Settings(settingsFile);
        std::cout << "Settings Loaded fromt file " << settingsFile << std::endl;
        cloud.reset(new pcl::PointCloud<PointType>);
    }

    pcl::PointCloud<PointType>::Ptr Capturer::Capture() {
        auto frame = camera.capture(settings);
        auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        cloud->width = data.width();
        cloud->height = data.height();
        cloud->is_dense = false;
        cloud->points.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i)
        {
            cloud->points[i].x = data(i).point.x;
            cloud->points[i].y = data(i).point.y;
            cloud->points[i].z = data(i).point.z;
            cloud->points[i].r = data(i).color.r;
            cloud->points[i].g = data(i).color.g;
            cloud->points[i].b = data(i).color.b;
        }

        return cloud;
    }

};

int main(int argc, char* argv[]) {

    string filename = "../../pcd/";
    if (argc > 1) {
        filename += argv[1];
    } else {
        filename += "Capture.pcd";
    }

    Capturer capturer("../../txt/set.yml");
    auto cloud = capturer.Capture();
    pcl::io::savePCDFileBinary(filename, *cloud);

    auto viewer = pcl::visualization::PCLVisualizer("Viewer");

    viewer.addPointCloud(cloud);

    viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

    std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
    std::cout << "Press q to exit the viewer application" << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}