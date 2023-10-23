#include "CreateModel.h"

#include <pcl/io/pcd_io.h>

// Select Point on cloud using shift + Right Click
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* pclObj)
{
    CreateModel* Viewer = (CreateModel*)pclObj;

    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }
    event.getPoint(x, y, z);
    PointType pickPoint1(x, y, z);
    PointType pickPoint2(x, y, z - 50);
    Viewer->pickPoints.push_back(pickPoint1);
    
    std::stringstream cubenameStream;
    cubenameStream << "cube" << Viewer->cubeCounter;
    Viewer->cubeCounter++;
    std::string cubename = cubenameStream.str();

    Viewer->viewer->addCube(x - 2, x + 2, y - 2, y + 2, z - 2, z + 2, 1, .5, 0, cubename);
    Viewer->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubename);
    
    std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

CreateModel::CreateModel(std::string filename) {

    // Setup the cloud pointer
    cloud.reset(new PointCloudType);

    // Fill the cloud with some points
    pcl::io::loadPCDFile<PointType>(filename, *cloud);
    cropBoxFilter.setInputCloud(cloud);
}

void CreateModel::Configure() {
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setSize(900, 1000);
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->addPointCloud(cloud, "cloud"); // included points
    viewer->setCameraPosition(0, 0, -40, 0, -1, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)this);
    viewer->resetCamera();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}

void CreateModel::filterCloud() {

}

void CreateModel::updateCloud() {

}

void CreateModel::writeConfig(std::string foldername) {

}