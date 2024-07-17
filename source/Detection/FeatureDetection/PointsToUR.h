#include <iostream>

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h> 
#include "PointsToRobot.h"

#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512

typedef pcl::PointXYZRGB PointType;

class PointsToUR {
    public:
        PointsToUR(std::vector<float> calibration, bool transform);
        ~PointsToUR();
        void SendPoints(std::vector<PointType> points);
    private:
        void Reconnect();
        void StartServer();
        void StartRobot();
        void TransformPoints(std::vector<PointType>& points);

        SOCKET ClientSocket = INVALID_SOCKET;
        SOCKET ListenSocket = INVALID_SOCKET;
        std::string port = "30000";
        char* robotIP = "192.168.1.205";
        std::string startCommand = "load socket.urp"; // command sent to ur to start control program
        int iResult;
        bool useTransform; // true to use transform from calibration board, false to use vector from cal.txt file
        std::vector<PointType> transformedPoints;
        std::vector<float> transformVector;
        float xOffset;
        float yOffset;
        float zOffset;
        float xFactor;
        float yFactor;
        float zFactor;
};