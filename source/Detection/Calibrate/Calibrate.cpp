#include "Calibrate.h"
//#include <vtkRenderWindow.h>

Calibrate::Calibrate(pcl::PointCloud<PointType>::Ptr scene) {
	Reset();
	cloud = scene;
	useCamera = false;
	ConnectToSocket();
}

Calibrate::Calibrate() {
	Reset();
	useCamera = true;
	std::cout << "Connecting to camera" << std::endl;
	camera = zivid.connectCamera();
	std::cout << "Connected" << std::endl;
	settings = Zivid::Settings("../../txt/set.yml");
	std::cout << "Settings Loaded from file\n" << std::endl;
	ConnectToSocket();
}

void Calibrate::Reset() {
	cloud.reset(new pcl::PointCloud<PointType>);
	cloud_filtered.reset(new pcl::PointCloud<PointType>);
	cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
	cloud_filtered2.reset(new pcl::PointCloud<PointType>);
	cloud_normals2.reset(new pcl::PointCloud<pcl::Normal>);
	coefficients_plane.reset(new pcl::ModelCoefficients);
	coefficients_sphere.reset(new pcl::ModelCoefficients);
	inliers_plane.reset(new pcl::PointIndices);
	inliers_sphere.reset(new pcl::PointIndices);
	cloud_sphere.reset(new pcl::PointCloud<PointType>);
}

void Calibrate::ReceivePoint() {
	iResult = recv(ConnectSocket, (char *)position, 28, 0);
	for (int i = 0; i < 7; i++) {
		std::cout << position[i] << endl;
	}
	PointType p;
	p.x = ((float)position[1])/10;
	p.y = ((float)position[2])/10;
	p.z = ((float)position[3])/10;
	robotPoints.push_back(p);
}

void Calibrate::CaptureScene() {
	auto frame = camera.capture(settings);
	auto pointCloud = frame.pointCloud();
	const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

	cloud->width = data.width();
	cloud->height = data.height();
	cloud->is_dense = false;
	cloud->points.resize(data.size());
	for (size_t i = 0; i < data.size(); ++i)
	{
		cloud->points[i].x = data(i).point.x;
		cloud->points[i].y = data(i).point.y;
		cloud->points[i].z = data(i).point.z;
		cloud->points[i].r = data(i).color.r;
		cloud->points[i].g = data(i).color.g;
		cloud->points[i].b = data(i).color.b;
	}

}

void Calibrate::ProcessScene() {
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(2);
	uniform_sampling.filter(*cloud);

	std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);
}

void Calibrate::FindSphere() {
	if(useCamera) CaptureScene();

	ProcessScene();

	// Create the segmentation object for sphere segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.8);
	seg.setMaxIterations(5000000);
	seg.setDistanceThreshold(.5);
	seg.setRadiusLimits(20, 30);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);

	// Obtain the sphere inliers and coefficients
	seg.segment(*inliers_sphere, *coefficients_sphere);
	std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;

	// Write the sphere inliers to cloud_sphere
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_sphere);
	extract.setNegative(false);
	extract.filter(*cloud_sphere);
	if (cloud_sphere->points.empty()) {
		std::cerr << "Can't find the sphere." << std::endl;
		exit(0);
	}
	std::cerr << "PointCloud representing the sphere: " << cloud_sphere->size() << " data points." << std::endl;

	PointType p;
	p.x = coefficients_sphere->values[0];
	p.y = coefficients_sphere->values[1];
	p.z = coefficients_sphere->values[2];
	cameraPoints.push_back(p);

}

void Calibrate::SendResponse() {
	std::string response = "next";
	iResult = send(ConnectSocket, response.c_str(), (int)strlen(response.c_str()), 0);
}

void Calibrate::LinearRegression(std::vector<float> xVals, std::vector<float> yVals, float &a, float &b) {
	float sum_x = 0, sum_x2 = 0, sum_y = 0, sum_xy = 0;
	int n = xVals.size();

	/* Calculating Required Sum */
	for (int i = 0; i < n; i++) {
		sum_x = sum_x + xVals[i];
		sum_x2 = sum_x2 + xVals[i] * xVals[i];
		sum_y = sum_y + yVals[i];
		sum_xy = sum_xy + xVals[i] * yVals[i];
	}
	
	/* Calculating a and b */
	b = ((float)n * sum_xy - sum_x * sum_y) / ((float)n * sum_x2 - sum_x * sum_x);
	a = (sum_y - b * sum_x) / (float)n;

	/* Displaying value of a and b */
	cout << "xVals" << endl;
	for (int i = 0; i < xVals.size(); i++) cout << "\t" << xVals[i] << endl;
	cout << "yVals" << endl;
	for (int i = 0; i < yVals.size(); i++) cout << "\t" << yVals[i] << endl;
	cout << "Calculated value of a is " << a << "and b is " << b << " n=" << n << endl;
	cout << "Equation of best fit line is: y = " << a << " + " << b << "x" << endl;
}

void Calibrate::CalculateCalibration() {
	std::vector<float> xCam, xRobo, yCam, yRobo, zCam, zRobo;
	std::cout << "Robot points" << endl;
	for (int i = 0; i < robotPoints.size(); i++) {
		cout << "\t";
		cout << robotPoints[i].x << " " << robotPoints[i].y << " " << robotPoints[i].z << endl;
		xRobo.push_back(robotPoints[i].x);
		yRobo.push_back(robotPoints[i].y);
		zRobo.push_back(robotPoints[i].z);
	}

	std::cout << "Camera points" << endl;
	for (int i = 0; i < cameraPoints.size(); i++) {
		cout << "\t";
		cout << cameraPoints[i].x << " " << cameraPoints[i].y << " " << cameraPoints[i].z << endl;
		xCam.push_back(cameraPoints[i].x);
		yCam.push_back(cameraPoints[i].y);
		zCam.push_back(cameraPoints[i].z);
	}

	LinearRegression(xCam, xRobo, xOffset, xFactor);
	LinearRegression(yCam, yRobo, yOffset, yFactor);
	LinearRegression(zCam, zRobo, zOffset, zFactor);

	/*xFactor = (robotPoints[1].x - robotPoints[0].x) / (cameraPoints[1].x - cameraPoints[0].x);
	yFactor = (robotPoints[3].y - robotPoints[2].y) / (cameraPoints[3].y - cameraPoints[2].y);
	zFactor = (robotPoints[5].z - robotPoints[4].z) / (cameraPoints[5].z - cameraPoints[4].z);

	xOffset = robotPoints[0].x - (xFactor * cameraPoints[0].x);
	yOffset = robotPoints[2].y - (yFactor * cameraPoints[2].y);
	zOffset = robotPoints[4].z - (zFactor * cameraPoints[4].z);*/
}

void Calibrate::WriteCalibration(std::string calFile) {
	std::ofstream calStream;
	calStream.open(calFile);
	calStream << xOffset << endl;
	calStream << yOffset << endl;
	calStream << zOffset << endl;
	calStream << xFactor << endl;
	calStream << yFactor << endl;
	calStream << zFactor << endl;
	calStream.close();
}

void Calibrate::VisualizeSphere() {
	viewer.reset(new pcl::visualization::PCLVisualizer("sphere Grouping"));
	//viewer->getRenderWindow()->GlobalWarningDisplayOff();
	viewer->setSize(900, 1000);
	viewer->setBackgroundColor(.3, .3, .3);
	viewer->setCameraPosition(0, 0, -50, 0, -1, 0);

	while (true) {
		viewer->addPointCloud(cloud, "scene_cloud");
		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(cloud_sphere, 255, 0, 0);
		viewer->addPointCloud(cloud_sphere, rotated_model_color_handler, "sphere_cloud");
		viewer->resetCamera();
		refreshViewer = false;
		while (!viewer->wasStopped() && !refreshViewer)
		{
			viewer->spinOnce();
		}
		viewer->removeAllPointClouds();
	}
	
}

void Calibrate::RefreshViewer() {
	refreshViewer = true;
}

void Calibrate::ConnectToSocket() {
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		exit(0);
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("localhost", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		exit(0);
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL;ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			exit(0);
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		printf("Unable to find server!\n");
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		exit(0);
	}

	std::string startMessage = "Connected";

	// Send an initial buffer
	iResult = send(ConnectSocket, startMessage.c_str(), (int)strlen(startMessage.c_str()), 0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		exit(0);
	}
	printf("Bytes Sent: %ld\n", iResult);

}