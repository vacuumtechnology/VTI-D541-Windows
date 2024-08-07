#include "PointsToRobot.h"

//PointsToRobot::PointsToRobot(Eigen::Matrix4f &transform) {
//    useTransform = true;
//
//    this->transform = transform;
//
//    ConnectToSocket();
//}

PointsToRobot::PointsToRobot(std::vector<float> calibration, bool transform) {
    useTransform = transform;

    if (transform) {
        this->transformVector = calibration;
    } else {
        if (calibration.size() != 6) {
            std::cout << "invalid calibration" << std::endl;
            exit(0);
        }
        xOffset = calibration[0];
        yOffset = calibration[1];
        zOffset = calibration[2];
        xFactor = calibration[3];
        yFactor = calibration[4];
        zFactor = calibration[5];

    }
    ConnectToSocket();

}

// constructor used for file output
PointsToRobot::PointsToRobot(std::vector<float> calibration, bool transform, std::string filename) {
    useTransform = transform;
    fileOutput = true;

    if (transform) {
        this->transformVector = calibration;
    } else {
        if (calibration.size() != 6) {
            std::cout << "invalid calibration" << std::endl;
            exit(0);
        }
        xOffset = calibration[0];
        yOffset = calibration[1];
        zOffset = calibration[2];
        xFactor = calibration[3];
        yFactor = calibration[4];
        zFactor = calibration[5];

    }
}

void PointsToRobot::WaitForHome() {
    iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
}

void PointsToRobot::TransformPoints(std::vector<std::pair<PointType, Eigen::Matrix3f>>&points){
    if (useTransform) {
        Eigen::Matrix4f transform;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transform(i, j) = transformVector[(4*i) + j];
            }
        }

        float factor = 1.006;
        for (int i = 0; i < points.size(); i++) {
            points[i].first.x *= factor;
            points[i].first.y *= factor;
            points[i].first.z *= factor;
        }

        // put points in cloud so transformation matrix can be used
        pcl::PointCloud<PointType> pickCloud;
        for (int i = 0; i < points.size(); i++) {
            std::cout << points[i].first.x << std::endl;
            pickCloud.push_back(points[i].first);
        }
        pcl::transformPointCloud(pickCloud, pickCloud, transform); // transform to robot base frame

        // store back in vector
        for (int i = 0; i < pickCloud.points.size(); i++) {
            points[i].first = pickCloud.points[i]; // TODO: not sure if this works, need to check if points in cloud are in same order they are added
        }
        for (int i = 0; i < points.size(); i++) {
            points[i].first.x = (int)(points[i].first.x * 10);
            points[i].first.y = (int)(points[i].first.y * 10);
            points[i].first.z = (int)(points[i].first.z * 10); // CHECK IF -z IS UP OR DOWN
            std::cout << "transformed point: " << points[i].first.x << " " << points[i].first.y << " " << points[i].first.z << std::endl;
        }
    } else {
        for (int i = 0; i < points.size(); i++) {
            points[i].first.x = (int)((-1 * points[i].first.x) + xOffset) * 10;
            points[i].first.y = (int)(points[i].first.y + yOffset) * 10;
            points[i].first.z = (int)(((-1 * points[i].first.z) + zOffset) + 2) * 10; // CHECK IF -z IS UP OR DOWN
            std::cout << "transformed point: " << points[i].first.x << " " << points[i].first.y << " " << points[i].first.z << std::endl;
        }
    }
}

void PointsToRobot::PointsToFile(std::vector<std::pair<PointType, Eigen::Matrix3f>> points, bool append) {
    std::ofstream myfile;
    std::string filename = "../../txt/points.txt";
    if (append) {
        myfile.open(filename, std::ofstream::out | std::ofstream::app);
    } else {
        myfile.open(filename, std::ofstream::out);
    }

    for (int i = 0; i < points.size(); i++) {
        myfile << points[i].first.x << "," << points[i].first.y << "," << points[i].first.z;
        Eigen::Vector3f rotationVector = points[i].second.eulerAngles(2, 1, 0);
        myfile << "," << rotationVector[0] << "," << rotationVector[1] << "," << rotationVector[2] << std::endl;
    }

    myfile.close();
}

void PointsToRobot::SendPoints(std::vector<std::pair<PointType, Eigen::Matrix3f>> points) {
    std::cout << "SendPoints called" << std::endl;

    TransformPoints(points);

    float x, y, z;
    int msg[7];
    std::string responseStr = "";
    PointType point;
    std::string startMessage = "go";

    iResult = send(ConnectSocket, startMessage.c_str(), startMessage.size(), 0);
    std::cout << "Start Message Sent to Robot" << std::endl;

    RobotMoving = true;
    int ind = -1;
    while (1) {

        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
        printf("Bytes received: %d\n", iResult);
        responseStr = recvbuf;

        std::string resend = "again";
        if (responseStr != resend) {
            ind++;
            if (ind >= points.size()) break;
            point = points[ind].first;
            msg[0] = 111;
            msg[1] = (int)point.x;
            msg[2] = (int)point.y;
            msg[3] = (int)point.z; // CHECK IF -z IS UP OR DOWN
            msg[4] = 30 * 10000;
            msg[5] = 0;
            msg[6] = 180 * 10000;
            
        } else {
            std::cout << "resending..." << std::endl;
        }


        iResult = send(ConnectSocket, (char*)msg, 28, 0);
        if (iResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ConnectSocket);
            WSACleanup();
            exit(0);
        }
        printf("sent: %d, %d, %d, %d\n", msg[0], msg[1], msg[2], msg[3]);

    }

    msg[0] = 222;
    iResult = send(ConnectSocket, (char*)msg, 16, 0);
    printf("Goodbye message sent\n");
    iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
    RobotMoving = false;
}


char * PointsToRobot::IntToBytes(int msg[]){
    char * c = (char *)malloc(16);
    for(int i = 0; i < 4; i++){
        c[4*i] = (msg[i] & 0xff000000) >> 24;
        c[4*i+1] = (msg[i] & 0x00ff0000) >> 16;
        c[4*i+2] = (msg[i] & 0x0000ff00) >> 8;
        c[4*i+3] = (msg[i] & 0x000000ff);
    }
    return c;
}

void PointsToRobot::ConnectToSocket() {

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        exit(0);
    }

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(servername.c_str(), DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        exit(0);
    }

    // Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            exit(0);
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        exit(0);
    }
    std::cout << "Client connected" << std::endl;

    std::string connectMsg = "connected";
    // Send an initial buffer
    iResult = send(ConnectSocket, connectMsg.c_str(), connectMsg.size(), 0);
    if (iResult == SOCKET_ERROR) {
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        exit(0);
    }

    printf("Bytes Sent: %ld\n", iResult);
}

// Destructor
PointsToRobot::~PointsToRobot() {
    iResult = shutdown(ConnectSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        exit(0);
    }
    closesocket(ConnectSocket);
    WSACleanup();
}