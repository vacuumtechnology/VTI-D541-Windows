#include "PointsToUR.h"

PointsToUR::PointsToUR(std::vector<float> calibration, bool transform){
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
    StartServer();
}

void PointsToUR::StartServer() {
    WSADATA wsaData;


    struct addrinfo* result = NULL;
    struct addrinfo hints;


    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return;
    }

    // Create a SOCKET for the server to listen for client connections.
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return;
    }

    // Setup the TCP listening socket
    iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    // No longer need server socket
    closesocket(ListenSocket);

    return;
}

void PointsToUR::StartRobot() {
    WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo* result = NULL,
        * ptr = NULL,
        hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(robotIP, DEFAULT_PORT, &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL;ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return;
        }

        // Connect to server.
        iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
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
        return;
    }


    // Send an initial buffer
    iResult = send(ConnectSocket, startCommand.c_str(), startCommand.size(), 0);
    if (iResult == SOCKET_ERROR) {
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return;
    }

    printf("Bytes Sent: %ld\n", iResult);

    // shutdown the connection since no more data will be sent
    iResult = shutdown(ConnectSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return;
    }

    // Receive until the peer closes the connection
    do {

        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
        if (iResult > 0)
            printf("Bytes received: %d\n", iResult);
        else if (iResult == 0)
            printf("Connection closed\n");
        else
            printf("recv failed with error: %d\n", WSAGetLastError());

    } while (iResult > 0);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

    return;
}

void PointsToUR::Reconnect() {
    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }
}

void PointsToUR::TransformPoints(std::vector<PointType>& points) {
    if (useTransform) {
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
            points[i].x = (int)(points[i].x * 10);
            points[i].y = (int)(points[i].y * 10);
            points[i].z = (int)(points[i].z * 10); // CHECK IF -z IS UP OR DOWN
            std::cout << "transformed point: " << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
        }
    } else {
        for (int i = 0; i < points.size(); i++) {
            points[i].x = (int)((-1 * points[i].x) + xOffset) * 10;
            points[i].y = (int)(points[i].y + yOffset) * 10;
            points[i].z = (int)(((-1 * points[i].z) + zOffset) + 2) * 10; // CHECK IF -z IS UP OR DOWN
            std::cout << "transformed point: " << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
        }

    }
}

void PointsToUR::SendPoints(std::vector<PointType> points) {

    TransformPoints(points); // replace with robot mounted camera point translation

    int iSendResult;
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;

    int numPoints = points.size();
    std::string responseStr;

    float rx = 0;
    float ry = 0;
    float rz = 180;

    while (true) {
        // Send number of pick points
        iSendResult = send(ClientSocket, (char *)&numPoints, 4, 0);
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            Reconnect();
            continue;
        }
        printf("Bytes sent: %d\n", iSendResult);
        break;

        // Receive confirmation
        iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
        if (iResult > 0) {
            responseStr = recvbuf;
            printf("Bytes received: %d\n", iResult);

            if (responseStr == "good") {
                break;
            } else {
                continue;
            }
        } else if (iResult == 0) {
            return;
        } else {
            printf("recv failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            Reconnect();
            continue;
        }
    }

    for (int i = 0; i < points.size(); i++) {
        std::stringstream ss;
        std::string pointStr;

        ss << "(" << points[i].x << "," << points[i].y << "," << points[i].z << "," << rx << "," << ry << "," << rz << ")";
        pointStr = ss.str();

        // Send pick point
        iSendResult = send(ClientSocket, pointStr.c_str(), pointStr.size(), 0);
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            Reconnect();
            continue;
        }
        printf("Bytes sent: %d\n", iSendResult);
        break;

        // Receive confirmation
        iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
        if (iResult > 0) {
            responseStr = recvbuf;
            printf("Bytes received: %d\n", iResult);
            if (responseStr == "good") {
                break;
            } else {
                continue;
            }
        } else if (iResult == 0) {
            return;
        } else {
            printf("recv failed with error: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            Reconnect();
            continue;
        }

    }
}

PointsToUR::~PointsToUR() {
    // shutdown the connection since we're done
    shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();
}