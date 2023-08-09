/*
Crops point cloud using coordinates from txt/startingDimensions.txt,
Allows user to adjust coordinates using command prompts,
Writes final dimensions to txt/newDimensions.txt
Usage:  ./VisualCrop.exe input.pcd
*/

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <thread>
#include <fstream>
#include <mutex>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector <float> dimensions;

bool changed = false;
bool stopViewer;
std::mutex mtx;

void view() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;


	auto viewer = pcl::visualization::PCLVisualizer("Viewer");
	vtkObject::GlobalWarningDisplayOff(); // Hide vtk warning window
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_color_handler(cloud, 100, 100, 100); // Set excluded points to grey
	viewer.setSize(1500, 750);

	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, scene_color_handler, "cloud"); // excluded points
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_out, "cloud_out"); // included points
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, .5, "cloud");

	viewer.addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 252, 252, 0);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");

	viewer.setCameraPosition(0, 50, -50, 0, -1, 0);
	viewer.resetCamera();

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);

		// Check for updated dimensions, update cropped cloud and box
		mtx.lock();
		if (changed) {
			viewer.updatePointCloud(cloud_out, "cloud_out");
			viewer.removeShape("cube");
			viewer.addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 252, 252, 0);
			changed = false;
			mtx.unlock();
			viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
		} else mtx.unlock();

		// Exit viewer when closed
		mtx.lock();
		if (stopViewer) {
			stopViewer = false;
			mtx.unlock();
			break;
		} else mtx.unlock();
	}
}

int main(int argc, char** argv) {

	std::string s;
	fstream dimFile;
	stopViewer = false;
	bool done = false;

	std::vector <std::string> dimensionPrompts = {"xmin: ", "xmax: ", "ymin: ", "ymax: ", "zmin: ", "zmax: "};

	dimFile.open("txt/startingDimensions.txt", ios::in);
	// Reads cropping dimensions from stdin or txt file if redirection is used
	for (int i = 0; i < 6; i++) {
		std::getline(dimFile, s);
		dimensions.push_back(atof(s.c_str()));
	}
	dimFile.close();

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		std::cout << "load failed" << endl;
		return -1;
	}

	//Define CropBox
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(true);
	cropBoxFilter.setInputCloud(cloud);
	std::thread viewThread(view);

	while (1) {
		Eigen::Vector4f min_pt(dimensions[0], dimensions[2], dimensions[4], 500.0f);
		Eigen::Vector4f max_pt(dimensions[1], dimensions[3], dimensions[5], 500.0f);

		std::cout << "test done" << endl;

		// Cropbox slighlty bigger then bounding box of points
		cropBoxFilter.setMin(min_pt);
		cropBoxFilter.setMax(max_pt);

		// Indices
		std::vector<int> indices;
		cropBoxFilter.filter(indices);

		std::cout << "box done" << endl;

		// Cloud
		cropBoxFilter.filter(*cloud_out);

		std::cout << "filter done" << endl;

		mtx.lock();
		changed = true;
		mtx.unlock();
		

		std::cout << "Current Dimensions - ";
		for (int i = 0; i < 6; i++) {
			std::cout << dimensionPrompts[i] << dimensions[i] << ", ";
		}
		cout << endl;

		for (int i = 0; i < 6; i++) {
			cout << dimensionPrompts[i];
			cin >> s;
			if (s == "n" || s == "c") continue;
			if (s == "q") {
				done = true;
				break;
			}
			dimensions[i] = atof(s.c_str());
		}

		if (done) break;
	}
	mtx.lock();
	stopViewer = true;
	mtx.unlock();
	viewThread.join();

	dimFile.open("txt/newDimensions.txt", ios::out);
	for (int i = 0; i < dimensions.size(); i++) {
		dimFile << dimensions[i] << endl;
	}
	dimFile.close();

	return 0;
}