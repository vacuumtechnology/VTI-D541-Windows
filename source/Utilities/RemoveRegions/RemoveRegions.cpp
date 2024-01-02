/*
Remove regions of points from a point cloud by creating boxes in areas where points should be removed

*/

#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <iostream>
#include <thread>
#include <condition_variable>

std::condition_variable cv;
std::mutex cv_m;
bool cubeAdded = false;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class RemoveRegions {
public:
    RemoveRegions(std::string filename);
    void Visualizer();
    void Configure();
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::vector <float> dimensions;
    int increment;
    bool cloudUpdated;
    bool pointSelected;

private:
    void updateCloud();
    void filterCloud();
    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
    PointCloudType::Ptr cloud_out;
    PointCloudType::Ptr cloud;
    std::string filename;
};

// Select Point on cloud using shift + Right Click
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* pclObj)
{
    RemoveRegions* obj = (RemoveRegions*)pclObj;

    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }

    event.getPoint(x, y, z);
    std::cout << "Point ( " << x << ", " << y << ", " << z << ")" << std::endl;

    obj->dimensions[0] = x - 2;
    obj->dimensions[1] = x + 2;
    obj->dimensions[2] = y - 2;
    obj->dimensions[3] = y + 2;
    obj->dimensions[4] = z - 2;
    obj->dimensions[5] = z + 2;
    obj->viewer->removeShape("cube");
    obj->viewer->addCube(obj->dimensions[0], obj->dimensions[1], obj->dimensions[2], obj->dimensions[3], obj->dimensions[4], obj->dimensions[5], 1, .5, 0);
    obj->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    obj->cloudUpdated = true;
    obj->pointSelected = true;

}

// visualizer keystroke handler for cropping step
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* pclObj) {
    RemoveRegions* obj = (RemoveRegions*)pclObj;

    if (event.getKeySym() == "v" && event.keyUp()) {
    }
    switch (event.getKeyCode()) {
    case '1':
        obj->dimensions[0] -= obj->increment;
        break;
    case '2':
        obj->dimensions[1] += obj->increment;
        break;
    case '3':
        obj->dimensions[2] -= obj->increment;
        break;
    case '4':
        obj->dimensions[3] += obj->increment;
        break;
    case '5':
        obj->dimensions[4] -= obj->increment;
        break;
    case '6':
        obj->dimensions[5] += obj->increment;
        break;
    case 'p':
        obj->increment = 1;
        break;
    case 'o':
        obj->increment = 10;
        break;
    }
    obj->cloudUpdated = true;
}

RemoveRegions::RemoveRegions(std::string filename) {
    // Setup the cloud pointer
    cloud.reset(new PointCloudType);
    cloud_out.reset(new PointCloudType);

    // Fill the cloud with some points
    pcl::io::loadPCDFile<PointType>(filename, *cloud);
    cloud_out = cloud;
    cropBoxFilter.setInputCloud(cloud);
    cropBoxFilter.setNegative(true);

    increment = 5;
    pointSelected = false;
    this->filename = filename;
    dimensions.resize(6);
}

void RemoveRegions::Configure() {
    std::string s;
    bool done = false;
    std::unique_lock<std::mutex> lk(cv_m);

    while (!done) {
        std::cout << "Select a point you want to remove" << std::endl;
        cv.wait(lk, [] { return cubeAdded; });

        while (true) {
            std::cout << "Adjust region within viewer. Enter 'd' here when done: ";
            cin >> s;
            if (s == "d") break;
        }




        while (true) {
            std::cout << "Remove another region? [y/n]: ";
            cin >> s;
            if (s == "y") {
                break;
            } else if (s == "n") {
                done = true;
                break;
            }
        }
        cubeAdded = false;
    }

    std::cout << "Saving file " << filename << endl;
    pcl::io::savePCDFileBinary(filename, *cloud_out);
    std::cout << "Saved " << filename << endl;

}

void RemoveRegions::filterCloud() {
    Eigen::Vector4f min_pt(dimensions[0], dimensions[2], dimensions[4], 500.0f);
    Eigen::Vector4f max_pt(dimensions[1], dimensions[3], dimensions[5], 500.0f);

    // Cropbox bounds set to dimensions
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);

    // Cloud
    cropBoxFilter.filter(*cloud_out);
}

void RemoveRegions::updateCloud() {
    viewer->updatePointCloud(cloud_out, "cloud_out");
    viewer->removeShape("cube");
    viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");

}

void RemoveRegions::Visualizer() {
    viewer.reset(new pcl::visualization::PCLVisualizer);
    viewer->getRenderWindow()->GlobalWarningDisplayOff();
    viewer->setSize(900, 1000);
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->setCameraPosition(0, 0, -40, 0, -1, 0);

    viewer->addPointCloud(cloud_out, "cloud_out");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_color_handler(cloud, 120, 120, 120); // Set excluded points to grey
    viewer->addPointCloud(cloud, scene_color_handler, "cloud"); // excluded points
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, .5, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_out");

    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)this);
    viewer->addText("Shift + Click to select a point\n\nBox Controls:\nxmin -> '1'\nxmax -> '2'\nymin -> '3'\nymax -> '4'\nzmin -> '5'\nzmax -> '6'\nPrecision mode -> 'p'\nLow precision -> 'o'", 0, 20, "text1");
    viewer->resetCamera();
    cloudUpdated = false;

    while (!viewer->wasStopped()) {
        if (pointSelected) {
            viewer->registerKeyboardCallback(keyboardCallback, (void*)this);
            pointSelected = false;
            cloudUpdated = true;
            std::lock_guard<std::mutex> lk(cv_m);
            cubeAdded = true;
            cv.notify_all();
        }

        if (cloudUpdated) {
            filterCloud();
            updateCloud();
            cloudUpdated = false;
        }
        
        viewer->spinOnce();
    }
}

int main(int argc, char* argv[]){
    std::string filename = argv[1];
    RemoveRegions obj(filename);

    std::thread sceneViewer(&RemoveRegions::Visualizer, obj);

    obj.Configure();

    sceneViewer.join();

    return 1;
}