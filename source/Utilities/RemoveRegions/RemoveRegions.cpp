/*
Remove regions of points from a point cloud by creating boxes in areas where points should be removed

*/

#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <thread>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class RemoveRegions {
public:
    RemoveRegions(std::string filename);
    void Visualizer();
    void Configure();
private:

    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
    PointCloudType::Ptr cloud_out;
    PointCloudType::Ptr cloud;
};

RemoveRegions::RemoveRegions(std::string filename) {
    // Setup the cloud pointer
    cloud.reset(new PointCloudType);
    cloud_out.reset(new PointCloudType);

    // Fill the cloud with some points
    pcl::io::loadPCDFile<PointType>(filename, *cloud);
    cropBoxFilter.setInputCloud(cloud);

    //increment = 10;
}

void RemoveRegions::Configure() {
    while (true) {
        std::cout << "Select a point you want to remove" << std::endl;

        std::cout << "Remove another region?" << std::endl;
    }


}

void RemoveRegions::Visualizer() {

}

int main(int argc, char* argv[]){
    std::string filename = argv[1];
    RemoveRegions obj(filename);

    std::thread sceneViewer(&RemoveRegions::Visualizer, obj);

    obj.Configure();


    return 1;
}