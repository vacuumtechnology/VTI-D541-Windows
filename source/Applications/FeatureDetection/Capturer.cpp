/*
Capture point clouds, with settings from file, from the Zivid camera, .
*/

#include "Capturer.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;

Capturer::Capturer(string settingsFile) {
    camera = zivid.connectCamera();
    settings = Zivid::Settings(settingsFile);
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
