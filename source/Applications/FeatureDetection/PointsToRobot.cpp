#include "PointsToRobot.h"

PointsToRobot::PointsToRobot(std::vector<float> calibration) {

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

    ConnectToSocket();
}

void PointsToRobot::WaitForHome() {
    iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
}

void PointsToRobot::SendPoints(std::vector<PointType> points) {
    std::cout << "SendPoints called" << std::endl;

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
            point = points[ind];
            msg[0] = 111;
            msg[1] = (int)((-1 * point.x) + xOffset) * 10;
            msg[2] = (int)(point.y + yOffset) * 10;
            msg[3] = (int)(((-1 * point.z) + zOffset) + 2) * 10; // CHECK IF -z IS UP OR DOWN
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