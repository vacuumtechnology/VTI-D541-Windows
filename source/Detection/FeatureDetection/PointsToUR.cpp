#include "PointsToUR.h"

using namespace ur_rtde;
using namespace std::chrono;

// interrupt flag
bool running = true;
void raiseFlag(int param){
    running = false;
}

PointsToUR::PointsToUR(std::vector<float> transform){
    // Setup parameters
    double rtde_frequency = 500.0; // Hz
    double dt = 1.0 / rtde_frequency; // 2ms
    uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
    int ur_cap_port = 50002;

    // ur_rtde realtime priorities
    int rt_receive_priority = 90;
    int rt_control_priority = 85;

    rtde_control = new RTDEControlInterface(robotIP, rtde_frequency, flags, ur_cap_port, rt_control_priority);
    rtde_receive = new RTDEReceiveInterface(robotIP, rtde_frequency, {}, true, false, rt_receive_priority);

    // Set application realtime priority
    RTDEUtility::setRealtimePriority(80);

    // Servo control parameters
    double lookahead_time = 0.1;
    double gain = 600;

    signal(SIGINT, raiseFlag);

    double time_counter = 0.0;
}

void PointsToUR::TransformPoints(std::vector<PointType>& points) {
    Eigen::Matrix4f transform;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform(i, j) = transformVector[(4 * i) + j];
        }
    }

    float factor = 1.006;
    for (int i = 0; i < points.size(); i++) {
        points[i].x *= factor;
        points[i].y *= factor;
        points[i].z *= factor;
    }

    // put points in cloud so transformation matrix can be used
    pcl::PointCloud<PointType> pickCloud;
    for (int i = 0; i < points.size(); i++) {
        pickCloud.push_back(points[i]);
    }
    pcl::transformPointCloud(pickCloud, pickCloud, transform); // transform to robot base frame
    points.clear();

    // store back in vector
    for (int i = 0; i < pickCloud.points.size(); i++) {
        points.push_back(pickCloud.points[i]);
    }
    for (int i = 0; i < points.size(); i++) {
        points[i].x = (int)(points[i].x);
        points[i].y = (int)(points[i].y);
        points[i].z = (int)(points[i].z);
        std::cout << "transformed point: " << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
    }
}

void PointsToUR::SendPoints(std::vector<std::pair<PointType, Eigen::Matrix3f>> points) {
    std::vector<double> currentPoint(6);
    for (int i = 0; i < points.size(); i++) {
        currentPoint[0] = points[i].first.x;
        currentPoint[1] = points[i].first.y;
        currentPoint[2] = points[i].first.z;
        currentPoint[3] = 0; // check rotations before testing
        currentPoint[4] = 0;
        currentPoint[5] = 0;
        rtde_control->moveL(currentPoint, vel, acc);
    }
}

PointsToUR::~PointsToUR() {
    delete(rtde_control);
    delete(rtde_receive);
}