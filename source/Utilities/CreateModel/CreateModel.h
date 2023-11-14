#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <pcl/common/common.h>
#include <string>
#include <thread>
#include <windows.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class CreateModel {
public:
	CreateModel(std::string filename);
	~CreateModel();
	void Configure();
	void Visualizer();
	pcl::visualization::PCLVisualizer::Ptr viewer;
	std::vector<PointType *> pickPoints;
	std::vector <float> dimensions;
	int cubeCounter = 0;
	int increment;
	bool cloudUpdated;
	PointCloudType::Ptr cloud_out;
private:

	void filterCloud();
	void updateCloud();

	void writeCloud();
	void writeConfig();

	PointCloudType::Ptr cloud;
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;

	std::string filename;
	std::string saveFile;
	std::string modelName;
	std::string path = "../../pcd/";

	bool switchScene;

	//Model parameters
	float model_ss;
	float descr_rad;
	float cg_size;
	float cg_thresh;
	float rf_rad;
	int max_objects;
	float out_thresh;
	int corr_thresh;
};