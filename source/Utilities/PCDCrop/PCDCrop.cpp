/*
Crops point cloud using coordinates from stdin
Usage: cat dim.txt | ./PCDCrop.exe input.pcd output.pcd
*/



#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <thread>

int main(int argc, char** argv) {

	std::string s;
	std::vector <float> dimensions;

	// Reads cropping dimensions from stdin or txt file if redirection is used
	for (int i = 0; i < 6; i++) {
		std::getline(cin, s);
		dimensions.push_back(atof(s.c_str()));
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		std::cout << "load failed" << endl;
		return -1;
	}

	//Define CropBox and dimensions
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(true);
	cropBoxFilter.setInputCloud(cloud);
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
	pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
	cropBoxFilter.filter(cloud_out);

	std::cout << "filter done" << endl;

	if (strcmp(argv[2], "-v") != 0) {
		pcl::io::savePCDFileBinary(argv[2], cloud_out);
		std::cout << "file saved" << endl;
	} else {
		auto viewer = pcl::visualization::PCLVisualizer("Viewer");
		viewer.setBackgroundColor(1, 1, 1);
		viewer.addPointCloud<pcl::PointXYZRGB>(cloud_out.makeShared());


		viewer.setCameraPosition(0, 0, -500, 0, -1, 0);

		std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
		std::cout << "Press q to exit the viewer application" << std::endl;
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	return 0;
}