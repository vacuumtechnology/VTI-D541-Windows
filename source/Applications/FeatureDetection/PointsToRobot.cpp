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

    CreateSocket();
}

void PointsToRobot::SendPoints(std::vector<PointType> points) {

    float x, y, z;
    int msg[7];
    std::string responseStr = "";
    PointType point;
    std::string startMessage = "go";

    iSendResult = send(ClientSocket, startMessage.c_str(), startMessage.size(), 0);

    RobotMoving = true;
    int ind = -1;
    while (1) {

        iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
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
            msg[3] = (int)(((-1 * point.z) + zOffset) + 5) * 10; // CHECK IF -z IS UP OR DOWN
            msg[4] = 30 * 10000;
            msg[5] = 0;
            msg[6] = 180 * 10000;
            
        } else {
            std::cout << "resending..." << std::endl;
        }


        iSendResult = send(ClientSocket, (char*)msg, 28, 0);
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            exit(0);
        }
        printf("sent: %d, %d, %d, %d\n", msg[0], msg[1], msg[2], msg[3]);


    }

    msg[0] = 222;
    iSendResult = send(ClientSocket, (char*)msg, 16, 0);
    printf("Goodbye message sent\n");
    iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
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

void PointsToRobot::CreateSocket() {
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        exit(0);
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        exit(0);

    }

    // Create a SOCKET for the server to listen for client connections.
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        exit(0);

    }

    // Setup the TCP listening socket
    iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        exit(0);

    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        exit(0);
    }

    std::cout << "Waiting for client to connect..." << std::endl;
    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        exit(0);
    }
}

// Destructor
PointsToRobot::~PointsToRobot() {
    closesocket(ClientSocket);
    closesocket(ListenSocket);
    WSACleanup();
}