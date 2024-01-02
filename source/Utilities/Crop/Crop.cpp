/*
Crops point cloud using coordinates from stdin
Usage: ./Crop.exe ../../pcd/input.pcd output
*/



#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <thread>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

class Crop {
public:
	Crop(std::string filename, std::string saveFile);
	void Visualizer();
	pcl::visualization::PCLVisualizer::Ptr viewer;
	std::vector<PointType*> pickPoints;
	std::vector <float> dimensions;
	int increment;
	bool cloudUpdated;
	bool closeViewer = false;
	PointCloudType::Ptr cloud_out;
private:

	void filterCloud();
	void updateCloud();

	void writeCloud();

	PointCloudType::Ptr cloud;
	PointCloudType::Ptr cloudx;
	PointCloudType::Ptr cloudy;
	pcl::PassThrough<pcl::PointXYZRGB> passX;
	pcl::PassThrough<pcl::PointXYZRGB> passY;
	pcl::PassThrough<pcl::PointXYZRGB> passZ;

	std::string filename;
	std::string saveFile;
	std::string modelName;
	std::string path = "../../pcd/";
};

// visualizer keystroke handler for cropping step
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* obj) {
	Crop* mod = (Crop*)obj;
	if (event.keyDown()) {
		switch (event.getKeyCode()) {
		case '1':
			mod->dimensions[0] -= mod->increment;
			break;
		case '2':
			mod->dimensions[0] += mod->increment;
			break;
		case '3':
			mod->dimensions[1] -= mod->increment;
			break;
		case '4':
			mod->dimensions[1] += mod->increment;
			break;
		case '5':
			mod->dimensions[2] -= mod->increment;
			break;
		case '6':
			mod->dimensions[2] += mod->increment;
			break;
		case '7':
			mod->dimensions[3] -= mod->increment;
			break;
		case '8':
			mod->dimensions[3] += mod->increment;
			break;
		case '9':
			mod->dimensions[4] -= mod->increment;
			break;
		case '0':
			mod->dimensions[4] += mod->increment;
			break;
		case '-':
			mod->dimensions[5] -= mod->increment;
			break;
		case '=':
			mod->dimensions[5] += mod->increment;
			break;
		case 'p':
			mod->increment = 1;
			break;
		case 'o':
			mod->increment = 10;
			break;
		case 'd':
			mod->closeViewer = true;
			break;
		}
		mod->cloudUpdated = true;
	}
}

// constructor
Crop::Crop(std::string filename, std::string saveFile) {

	// Setup the cloud pointer
	cloud.reset(new PointCloudType);
	cloudx.reset(new PointCloudType);
	cloudy.reset(new PointCloudType);
	cloud_out.reset(new PointCloudType);
	this->saveFile = saveFile;

	// Fill the cloud with some points
	pcl::io::loadPCDFile<PointType>(filename, *cloud);
	passX.setInputCloud(cloud);
	passX.setFilterFieldName("x");
	passY.setInputCloud(cloudx);
	passY.setFilterFieldName("y");
	passZ.setInputCloud(cloudy);
	passZ.setFilterFieldName("z");

	dimensions.resize(6);
	dimensions = { -200, 200, -200, 200, 500, 900 }; // initial cropbox dimensions
	filterCloud();

	increment = 10;
}

void Crop::filterCloud() {
	cout << "before: " << cloud_out->size() << endl;
	for (int i = 0; i < dimensions.size(); i++) {
		std::cout << dimensions[i] << endl;
	}
	passX.setFilterLimits(dimensions[0], dimensions[1]);
	passY.setFilterLimits(dimensions[2], dimensions[3]);
	passZ.setFilterLimits(dimensions[4], dimensions[5]);
	passX.filter(*cloudx);
	passY.filter(*cloudy);
	passZ.filter(*cloud_out);
	cout << "after: " << cloud_out->size() << endl;
}

void Crop::updateCloud() {
	viewer->updatePointCloud(cloud_out, "cloud_out");
	viewer->removeShape("cube");
	viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");

}

void Crop::writeCloud() {
	std::string filename = path + saveFile + ".pcd";
	pcl::io::savePCDFileBinary(filename, *cloud_out);
	std::cout << "file saved" << endl;
}

void Crop::Visualizer() {
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setSize(900, 1000);
	viewer->setBackgroundColor(.3, .3, .3);
	viewer->setCameraPosition(0, 0, -40, 0, -1, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_color_handler(cloud, 120, 120, 120); // Set excluded points to grey
	viewer->addPointCloud(cloud, scene_color_handler, "cloud"); // excluded points

	viewer->addPointCloud(cloud_out, "cloud_out"); // included points
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, .5, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_out");
	viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
	viewer->addText("'1' <- xmin -> '2'\n'3' <- xmax -> '4'\n'5' <- ymin -> '6'\n'7' <- ymax -> '8'\n'9' <- zmin -> '0'\n'-' <- zmax -> '='\nPrecision mode -> 'p'\nLow precision -> 'o'\nDone ->'d'", 0, 20, "text1");
	viewer->registerKeyboardCallback(keyboardCallback, (void*)this);
	viewer->resetCamera();
	cloudUpdated = false;
	while (!viewer->wasStopped() && !closeViewer) {
		while (!cloudUpdated && !closeViewer) {
			viewer->spinOnce();
		}
		filterCloud();
		updateCloud();
		cloudUpdated = false;
	}
	writeCloud();
}

int main(int argc, char** argv) {

	std::string s = argv[1];
	std::string s2 = argv[2];
	Crop obj(s, s2);
	std::cout << "here" << endl;
	obj.Visualizer();


	return 0;
}