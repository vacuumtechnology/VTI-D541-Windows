/*
Capture point clouds, with color, from the Zivid camera, save it to PCD file format, and visualize it.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <string>
#include <thread>

namespace
{
    template<typename T>
    pcl::PointCloud<T> addDataToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data)
    {
        auto pointCloud = pcl::PointCloud<T>();

        pointCloud.width = data.width();
        pointCloud.height = data.height();
        pointCloud.is_dense = false;
        pointCloud.points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB> convertToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data)
    {
        return addDataToPCLPointCloud<pcl::PointXYZRGB>(data);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> convertToPCLPointCloud(
        const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data,
        const Zivid::Array2D<Zivid::NormalXYZ> &normals)
    {
        auto pointCloud = addDataToPCLPointCloud<pcl::PointXYZRGBNormal>(data);

        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud.points[i].normal_x = normals(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].normal_y = normals(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].normal_z = normals(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        return pointCloud;
    }

    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGBNormal>(pointCloud);

        const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pointCloud, normalsSkipped, 1, "normals");
    }

    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud);
    }

    template<typename T>
    void visualizePointCloudPCL(const pcl::PointCloud<T> &pointCloud)
    {
        auto viewer = pcl::visualization::PCLVisualizer("Viewer");
        viewer.setBackgroundColor(255, 255, 255);
        addPointCloudToViewer(viewer, pointCloud.makeShared());
        

        viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    template<typename T>
    void visualizeAndSavePointCloudPCL(const pcl::PointCloud<T> &pointCloud)
    {
        std::cout << "Visualizing PCL point cloud" << std::endl;
        visualizePointCloudPCL(pointCloud);

        std::string pointCloudFile = name;
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        pcl::io::savePCDFileBinary(pointCloudFile, pointCloud);
        std::cerr << "Saved " << pointCloud.points.size() << " points" << std::endl;
    }

} // namespace

void SavePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, std::string name)
{
    std::string pointCloudFile = name;
    std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
    pcl::io::savePCDFileBinary(pointCloudFile, pointCloud);
    std::cerr << "Saved " << pointCloud.points.size() << " points" << std::endl;


    std::cout << "Visualizing PCL point cloud" << std::endl;
    visualizePointCloudPCL(pointCloud);

}

int main(int argc, char **argv)
{
    try
    {

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        // Using Capture Assistant to find best settings
#pragma region settings
        const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1200 } }
        };

        std::cout << "Running Capture Assistant with parameters:\n" << suggestSettingsParameters << std::endl;
        auto settings = Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters);

        std::cout << "Settings suggested by Capture Assistant:" << std::endl;
        std::cout << settings.acquisitions() << std::endl;

        std::cout << "Manually configuring processing settings (Capture Assistant only suggests acquisition settings)"
            << std::endl;
        const auto processing = Zivid::Settings::Processing{
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Experimental::Mode::global,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 }
        };
        settings.set(processing);
#pragma endregion settings

        // Capturing
        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        // Save to zdf file
        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to zdf file: " << dataFile << std::endl;
        //frame.save(dataFile);

        // Convert to PCD and open
        std::cout << "Converting Zivid point cloud to PCL format" << std::endl;
        const auto pointCloudPCL = convertToPCLPointCloud(data);

        SavePointCloud(pointCloudPCL, argv[1]);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
