/*
Crops point cloud using coordinates from stdin
Usage: cat dim.txt | ./Capture.exe input.pcd output.pcd
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

pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
bool stopViewer;
std::mutex mtx;

void view() {
	auto viewer = pcl::visualization::PCLVisualizer("Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_out.makeShared());


	viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
	std::vector <float> dimensions;
	fstream dimFile;
	stopViewer = false;
	bool done = false;

	std::vector <std::string> dimensionPrompts = {"xmin: ", "ymin: ", "zmin: ", "xmax: ", "ymax: ", "zmax: "};

	dimFile.open("txt/dim.txt", ios::in);
	// Reads cropping dimensions from stdin or txt file if redirection is used
	for (int i = 0; i < 6; i++) {
		std::getline(dimFile, s);
		dimensions.push_back(atof(s.c_str()));
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		std::cout << "load failed" << endl;
		return -1;
	}

	//Define CropBox
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(true);
	cropBoxFilter.setInputCloud(cloud);


	while (1) {
		Eigen::Vector4f min_pt(dimensions[0], dimensions[1], dimensions[2], 500.0f);
		Eigen::Vector4f max_pt(dimensions[3], dimensions[4], dimensions[5], 500.0f);

		std::cout << "test done" << endl;

		// Cropbox slighlty bigger then bounding box of points
		cropBoxFilter.setMin(min_pt);
		cropBoxFilter.setMax(max_pt);

		// Indices
		std::vector<int> indices;
		cropBoxFilter.filter(indices);

		std::cout << "box done" << endl;

		// Cloud
		cropBoxFilter.filter(cloud_out);

		std::cout << "filter done" << endl;

		std::thread viewThread(view);

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

		mtx.lock();
		stopViewer = true;
		mtx.unlock();
		viewThread.join();
		if (done) break;
	}

	return 0;
}