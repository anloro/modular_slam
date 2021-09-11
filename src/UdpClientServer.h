/**
 * @file   UdpClientServer.h
 * @brief  UDP communication protocol.
 * @author Ángel Lorente Rogel
 * @date   05/09/2021
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <cstdint>
#include <array>

#define MAX_GRAPH_SIZE 1000

namespace anloro{

struct PoseData {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

struct UncertaintyData {
    float sigmaX;
    float sigmaY;
    float sigmaZ;
    float sigmaRoll;
    float sigmaPitch;
    float sigmaYaw;
}; 

struct RefFrameData {
    PoseData pose;
}; 

struct KeyFrameData {
    double timeStamp; 
    int id;
    PoseData pose;
}; 

struct LandmarkData {
    int landMarkId;
    PoseData pose;
    UncertaintyData unc;
}; 

struct PoseConstraintData {
    int idFrom;
    int idTo;
    PoseData pose;
    UncertaintyData unc;
}; 

union GraphElement
{
    RefFrameData refFrame;
    KeyFrameData keyFrame;    
    LandmarkData landmark; 
    PoseConstraintData poseFactor;   
    std::array<KeyFrameData, MAX_GRAPH_SIZE> graph;
};

struct MsgUdp {
    char type; // a: RefFrame, b: KeyFrame, c: LandMark, d: PoseFactor
    GraphElement element;
    int size;
};

// Server 
class UdpServer
{
    public:
        UdpServer();
        // Member functions
        int Receive(void * buffer);
        int Send(MsgUdp msg);

    protected: 
        // void* buffer; // Reception buffer
        int port = 1153;
        int fd; // our socket
        struct sockaddr_in myaddr; // our address
        struct sockaddr_in remaddr; // remote address
        socklen_t addrlen = sizeof(remaddr); // length of addresses
        int slen=sizeof(remaddr);

};

// Client
class UdpClient
{
    public:
        UdpClient();
        // Member functions
        int Send(MsgUdp msg);
        int Receive(void * buffer);
    
    protected: 
        struct sockaddr_in myaddr, remaddr;
        int fd, slen=sizeof(remaddr);
        int port = 1153;
        const char *server = "127.0.0.1"; // server address
        socklen_t addrlen = sizeof(remaddr); // length of addresses

};

} // namespace anloro