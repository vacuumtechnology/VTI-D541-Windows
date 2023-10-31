#include "CreateModel.h"
#include <pcl/io/pcd_io.h>
#include <mutex>

std::mutex mtx;

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

void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* obj) {
    CreateModel* mod = (CreateModel*)obj;
    if (event.getKeySym() == "v" && event.keyUp()) {
    }
}

CreateModel::CreateModel(std::string filename) {

    // Setup the cloud pointer
    cloud.reset(new PointCloudType);
    cloud_out.reset(new PointCloudType);

    // Fill the cloud with some points
    pcl::io::loadPCDFile<PointType>(filename, *cloud);
    cropBoxFilter.setInputCloud(cloud);

    dimensions.resize(6);
    dimensions = { -200, 200, -200, 200, 500, 900 };
}

void CreateModel::Configure() {}

void CreateModel::Visualizer() {
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setSize(900, 1000);
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->addPointCloud(cloud, "cloud"); // included points
    viewer->setCameraPosition(0, 0, -40, 0, -1, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    viewer->addText("'1' <- xmin -> '2'\n'3' <- xmax -> '4'\n'5' <- ymin -> '6'\n'7' <- ymax -> '8'\n'9' <- zmin -> '0'\n'-' <- zmax -> '='", 0, 0);
    viewer->resetCamera();
    switchScene = false;
    cloudUpdated = false;
    while (!viewer->wasStopped() && !switchScene) {
        while (!cloudUpdated) {
            viewer->spinOnce();
        }
        filterCloud();
        updateCloud();
        cloudUpdated = false;
    }

    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)this);
    viewer->resetCamera();
    switchScene = false;
    while (!viewer->wasStopped() && !switchScene) {

    }
}

void CreateModel::filterCloud() {
    Eigen::Vector4f min_pt(dimensions[0], dimensions[2], dimensions[4], 500.0f);
    Eigen::Vector4f max_pt(dimensions[1], dimensions[3], dimensions[5], 500.0f);

    // Cropbox slighlty bigger then bounding box of points
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);

    // Indices
    std::vector<int> indices;
    cropBoxFilter.filter(indices);

    // Cloud
    cropBoxFilter.filter(*cloud_out);
}

void CreateModel::updateCloud() {
    viewer->updatePointCloud(cloud_out, "cloud_out");
    viewer->removeShape("cube");
    viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");

}

void CreateModel::writeConfig(std::string foldername) {

}