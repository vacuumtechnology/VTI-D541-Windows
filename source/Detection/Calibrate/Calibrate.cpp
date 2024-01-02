#include "Calibrate.h"

Calibrate::Calibrate(pcl::PointCloud<PointType>::Ptr scene, int mode) {
	Reset();
	cloud = scene;
	useCamera = false;
	this->mode = mode;
	ConnectToSocket();
}

Calibrate::Calibrate(int mode) {
	Reset();
	useCamera = true;
	this->mode = mode;
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

	if (mode == 3) {
		/*std::cout << "Detecting checkerboard in point cloud" << std::endl;
		const auto detectionResult = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());

		if (detectionResult.valid())
		{
			std::cout << "Calibration board detected " << std::endl;
			handEyeInput.emplace_back(Zivid::Calibration::HandEyeInput{ robotPose, detectionResult });
		} else
		{
			std::cout
				<< "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
				<< std::endl;
		}*/
	} else {
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
	
	

}

void Calibrate::ProcessScene() {
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(.2);
	uniform_sampling.filter(*cloud);

	std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);
}

void Calibrate::FindDetectionObject() {
	if(useCamera) CaptureScene();

	ProcessScene();

	// Create the segmentation object for sphere segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.8);
	seg.setMaxIterations(5000000);
	seg.setDistanceThreshold(.3);
	seg.setRadiusLimits(24, 26);
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

void Calibrate::RegisterClouds() {
	pcl::PointCloud<PointType>::Ptr cameraCloud;
	pcl::PointCloud<PointType>::Ptr robotCloud;
	cameraCloud.reset(new pcl::PointCloud<PointType>);
	robotCloud.reset(new pcl::PointCloud<PointType>);
	for (int i = 0; i < cameraPoints.size(); i++) {
		cameraCloud->push_back(cameraPoints[i]);
		robotCloud->push_back(robotPoints[i]);
	}
	pcl::io::savePCDFileBinary("../../pcd/cameraCloud.pcd", *cameraCloud);
	pcl::io::savePCDFileBinary("../../pcd/robotCloud.pcd", *robotCloud);


	// Estimate cloud normals
	cout << "Computing source cloud normals\n";
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cameraNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_XYZRGB(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setInputCloud(cameraCloud);
	ne.setSearchMethod(tree_XYZRGB);
	ne.setKSearch(15);
	ne.compute(*cameraNormals);

	cout << "Computing target cloud normals\n";
	pcl::PointCloud<pcl::PointNormal>::Ptr robotNormals(new pcl::PointCloud<pcl::PointNormal>);
	ne.setInputCloud(robotCloud);
	ne.compute(*robotNormals);

	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cameraCloud);
	fpfh.setInputNormals(cameraNormals);
	fpfh.setSearchMethod(tree_XYZRGB);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cameraFeatures(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.setRadiusSearch(8.8);
	fpfh.compute(*cameraFeatures);
	cout << "Computed " << cameraFeatures->size() << " FPFH features for source cloud\n";

	fpfh.setInputCloud(robotCloud);
	fpfh.setInputNormals(robotNormals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr roboFeatures(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.compute(*roboFeatures);
	cout << "Computed " << roboFeatures->size() << " FPFH features for target cloud\n";

	// Sample Consensus Initial Alignment parameters (explanation below)
	const float min_sample_dist = 0.025f;
	const float max_correspondence_dist = 1.0f;
	const int nr_iters = 50000;

	// ICP parameters (explanation below)
	const float max_correspondence_distance = 100.0;
	const float outlier_rejection_threshold = 10.0f;
	const float transformation_epsilon = 5.0f;
	const int max_iterations = 10000;

	pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac_ia;
	sac_ia.setMinSampleDistance(min_sample_dist);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
	sac_ia.setMaximumIterations(nr_iters);

	sac_ia.setInputSource(cameraCloud);
	sac_ia.setSourceFeatures(cameraFeatures);

	sac_ia.setInputTarget(robotCloud);
	sac_ia.setTargetFeatures(roboFeatures);

	pcl::PointCloud<PointType> registration_output;
	sac_ia.align(registration_output);

	auto initial_alignment = sac_ia.getFinalTransformation();
	cout << "initial alignment found" << endl;

	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
	icp.setTransformationEpsilon(transformation_epsilon);
	icp.setMaximumIterations(max_iterations);

	pcl::PointCloud<PointType>::Ptr source_points_transformed(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cameraCloud, *source_points_transformed, initial_alignment);

	icp.setInputSource(source_points_transformed);
	icp.setInputTarget(robotCloud);

	icp.align(registration_output);

	transformMatrix = (icp.getFinalTransformation() * initial_alignment);
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

	if (mode == 1) {
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
	} else if(mode == 2){
		RegisterClouds();
	} else {

	}
}

void Calibrate::WriteCalibration(std::string calFile) {
	if (mode == 1) {
		std::ofstream calStream;
		calStream.open(calFile);
		calStream << xOffset << endl;
		calStream << yOffset << endl;
		calStream << zOffset << endl;
		calStream << xFactor << endl;
		calStream << yFactor << endl;
		calStream << zFactor << endl;
		calStream.close();
	} else if(mode == 2){
		std::ofstream calStream;
		calStream.open("../../txt/regtransform.cal");
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				calStream << transformMatrix(i, j) << " ";
			}
			calStream << std::endl;
		}
		calStream.close();
	} else {

	}
	
}

void Calibrate::VisualizeSphere() {
	viewer.reset(new pcl::visualization::PCLVisualizer("sphere Grouping"));
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