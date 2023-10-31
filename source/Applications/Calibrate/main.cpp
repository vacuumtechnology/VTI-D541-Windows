#include "Calibrate.h"
#include <thread>

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char* argv[])
{
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	Calibrate *cal;

	if (argc > 1) {
		string sceneFile = argv[1];
		if (pcl::io::loadPCDFile(sceneFile, *scene) < 0) {
			cout << "Error loading scene cloud." << endl;
			return (-1);
		}
		cal = new Calibrate(scene);
	} else {
		cal = new Calibrate();
	}

	thread sceneViewer(&Calibrate::VisualizeSphere, cal);

	for (int i = 0; i < 10; i++) {
		cal->ReceivePoint();
		cal->FindSphere();
		cal->SendResponse();
		cal->RefreshViewer();
	}

	cal->CalculateCalibration();
	cal->WriteCalibration("../../txt/cal.txt");

	sceneViewer.join();
	
	return (0);
}