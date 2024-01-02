#undef UNICODE
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

//#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "5001"

typedef pcl::PointXYZRGB PointType;


class PointsToRobot {
public:

	//Winsock vars
	WSADATA wsaData;
	int iResult;
	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo* result = NULL;
	struct addrinfo* ptr = NULL;
	struct addrinfo hints;
	char recvbuf[DEFAULT_BUFLEN];
	int recvbuflen = DEFAULT_BUFLEN;
	std::string servername = "localhost";

	PointsToRobot(std::vector<float> calibration, bool transform);
	~PointsToRobot();

	void WaitForHome();

	void SendPoints(std::vector<PointType> points);

private:
	bool useTransform; // true to use transform from calibration board, false to use vector from cal.txt file
	float xOffset;
	float yOffset;
	float zOffset;
	float xFactor;
	float yFactor;
	float zFactor;
	bool useRobot;
	bool RobotMoving = false;
	std::vector<PointType> transformedPoints;
	std::vector<float> transformVector;

	void ConnectToSocket();
	void TransformPoints(std::vector<PointType>& points);
	char *IntToBytes(int msg[]);
};

