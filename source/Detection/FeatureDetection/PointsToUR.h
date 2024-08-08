#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <vector>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h> 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef pcl::PointXYZRGB PointType;

class PointsToUR {
    public:
        PointsToUR(std::vector<float> transform);
        ~PointsToUR();
        void SendPoints(std::vector<std::pair<PointType, Eigen::Matrix3f>> points);
    private:
        void TransformPoints(std::vector<PointType>& points);

        
        std::string port = "30000";
        char* robotIP = "192.168.1.205";

        ur_rtde::RTDEControlInterface* rtde_control;
        ur_rtde::RTDEReceiveInterface* rtde_receive;

        bool useTransform; // true to use transform from calibration board, false to use vector from cal.txt file
        std::vector<PointType> transformedPoints;
        std::vector<float> transformVector;
        float xOffset;
        float yOffset;
        float zOffset;
        float xFactor;
        float yFactor;
        float zFactor;

        // Move parameters
        double vel = 0.5;
        double acc = 0.5;
};