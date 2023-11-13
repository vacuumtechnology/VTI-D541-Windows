/*
Read point cloud from PCL file and visualize it.

The PCD file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkRenderWindow.h>
#include <iostream>
#include <thread>

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "11111"

int main(int argc, char **argv)
{
    std::string pointCloudFile;
    void* parentID;
    pointCloudFile = "../../pcd/Capture.pcd";
    std::string servername = "localhost";

    WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo* result = NULL,
        * ptr = NULL,
        hints;
    const char* sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(servername.c_str(), DEFAULT_PORT, &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL;ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
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
        return 1;
    }

    // Send an initial buffer
    iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
    if (iResult == SOCKET_ERROR) {
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return 1;
    }

    printf("Bytes Sent: %ld\n", iResult);

    // shutdown the connection since no more data will be sent
    iResult = shutdown(ConnectSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return 1;
    }

    iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
    if (iResult > 0)
        printf("received: %s\n", recvbuf);
    else if (iResult == 0)
        printf("Connection closed\n");
    else
        printf("recv failed with error: %d\n", WSAGetLastError());

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();


    int handle = atoi(recvbuf);
    std::cout << "handle: " << handle << endl;

        
    std::cout << "Reading PCD point cloud from file: " << pointCloudFile << std::endl;

    auto pointCloudPCL = pcl::PointCloud<pcl::PointXYZRGB>();

    pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, pointCloudPCL);
    std::cout << "Loaded " << pointCloudPCL.width * pointCloudPCL.height << " points" << std::endl;

        
    auto viewer = pcl::visualization::PCLVisualizer("Viewer");
    auto window = viewer.getRenderWindow();
    //int* id = (int*)window->GetGenericWindowId();
    //cout << "id: " << *id << endl;
    window->SetParentId((void*)handle);

    viewer.addPointCloud<pcl::PointXYZRGB>(pointCloudPCL.makeShared());

    viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

    std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
    std::cout << "Press q to exit the viewer application" << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return EXIT_SUCCESS;
}
