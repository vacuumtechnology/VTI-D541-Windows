#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <pcl/point_types.h>

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

	PointsToRobot(std::vector<float> calibration);
	PointsToRobot(Eigen::Matrix4f transform);
	~PointsToRobot();

	void WaitForHome();

	void SendPoints(std::vector<PointType> points);

private:
	bool useBoard; // true to use transform from calibration board, false to use vector from cal.txt file
	Eigen::Matrix4f transform;
	float xOffset;
	float yOffset;
	float zOffset;
	float xFactor;
	float yFactor;
	float zFactor;
	bool useRobot;
	bool RobotMoving = false;
	std::vector<PointType> transformedPoints;

	void ConnectToSocket();
	char *IntToBytes(int msg[]);
};

