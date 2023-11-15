#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class ColorFilter {
public:
    ColorFilter(PointCloudType::Ptr cloud);
    PointCloudType::Ptr Filter();
    void ShowResult();
    int minr = -1;
    int ming = -1;
    int minb = -1;
    int maxr = -1;
    int maxg = -1;
    int maxb = -1;

private:
    PointCloudType::Ptr cloud;
    PointCloudType::Ptr cloud_out;
};

ColorFilter::ColorFilter(PointCloudType::Ptr cloud) {
    this->cloud = cloud;
    this->cloud_out.reset(new PointCloudType);
}

PointCloudType::Ptr ColorFilter::Filter() {
    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());

    int c = 0;
    if (minr > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition1(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, minr));
        color_cond->addComparison(red_condition1);
    }
    if (ming > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition1(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, ming));
        color_cond->addComparison(green_condition1);
    }
    if (minb > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition1(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, minb));
        color_cond->addComparison(blue_condition1);
    }
    if (maxr > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition2(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, maxr));
        color_cond->addComparison(red_condition2);
    }
    if (maxg > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition2(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, maxg));
        color_cond->addComparison(green_condition2);
    }
    if (maxb > 0) {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition2(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, maxb));
        color_cond->addComparison(blue_condition2);
    }
    std::cout << c << " points removed" << std::endl;

    // Build the filter
    color_filter.setInputCloud(cloud);
    color_filter.setCondition(color_cond);
    color_filter.filter(*cloud_out);
    return cloud_out;


}

void ColorFilter::ShowResult() {
    auto viewer = pcl::visualization::PCLVisualizer("Viewer");

    viewer.addPointCloud(cloud_out);
    viewer.setCameraPosition(0, 0, -100, 0, -1, 0);
    viewer.getRenderWindow()->GlobalWarningDisplayOff();
    viewer.setSize(900, 1000);
    viewer.setBackgroundColor(.3, .3, .3);
    viewer.resetCamera();

    std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
    std::cout << "Press q to exit the viewer application" << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}

int main(int argc, char* argv[]){
    PointCloudType::Ptr cloud;
    cloud.reset(new PointCloudType);
    std::string filein = argv[1];
    std::string fileout = argv[2];
    pcl::io::loadPCDFile<PointType>(filein, *cloud);
    std::cout << "out size: " << cloud->size() << endl;

    ColorFilter obj(cloud);

    obj.minb = 100;

    cloud = obj.Filter();
    std::cout << "out size: " << cloud->size() << endl;

    pcl::io::savePCDFileBinary(fileout, *cloud);

    obj.ShowResult();

    return 0;
}