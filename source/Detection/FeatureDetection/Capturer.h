#include <Zivid/Zivid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <iostream>

typedef pcl::PointXYZRGB PointType;

class Capturer {
private:
    Zivid::Application zivid;
    Zivid::Settings settings;
    Zivid::Camera camera;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;;

public:
    Capturer(std::string settingsFile);

    pcl::PointCloud<PointType>::Ptr Capture();

};