#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/uniform_sampling.h>
#include <Zivid/Zivid.h>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "5002"

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class Calibrate {
public:
	Calibrate();
	Calibrate(pcl::PointCloud<PointType>::Ptr scene);

	void ReceivePoint();
	void FindSphere();
	void SendResponse();
	void CalculateCalibration();
	void WriteCalibration(std::string calFile);
	void VisualizeSphere();
	void RefreshViewer();
	void Reset();
private:
	void ConnectToSocket();
	void CaptureScene();
	void ProcessScene();

	Zivid::Application zivid;
	Zivid::Settings settings;
	Zivid::Camera camera;

	pcl::PCDReader reader;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	pcl::PassThrough<PointType> pass;
	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointType> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointType>::Ptr tree;

	// Datasets
	pcl::PointCloud<PointType>::Ptr cloud;
	pcl::PointCloud<PointType>::Ptr cloud_filtered;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	pcl::PointCloud<PointType>::Ptr cloud_filtered2;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;
	pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_sphere;
	pcl::PointIndices::Ptr inliers_plane, inliers_sphere;
	pcl::PointCloud<PointType>::Ptr cloud_sphere;

	std::vector<PointType> cameraPoints;
	std::vector<PointType> robotPoints;

	int xOffset;
	int yOffset;
	int zOffset;
	int xFactor;
	int yFactor;
	int zFactor;

	bool refreshViewer = true;
	bool useCamera;

	//Winsock vars
	WSADATA wsaData;
	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo* result = NULL,
		* ptr = NULL,
		hints;
	const char* sendbuf = "this is a test";
	char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_BUFLEN;
	int position[7];
};