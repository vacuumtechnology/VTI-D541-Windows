#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <string>
#include <thread>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class CreateModel {
public:
	CreateModel(std::string filename);
	void Configure();
	void Visualizer();
	pcl::visualization::PCLVisualizer::Ptr viewer;
	std::vector<PointType> pickPoints;
	std::vector <float> dimensions;
	int cubeCounter = 0;
private:

	void filterCloud();
	void updateCloud();
	void writeConfig(std::string foldername);

	PointCloudType::Ptr cloud;
	PointCloudType::Ptr cloud_out;
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;

	std::string filename;
	std::string saveFile;

	bool switchScene;
	bool cloudUpdated;
};