// Server side implementation of UDP client-server model
#include "UdpClientServer.h"

using namespace anloro;


anloro::UdpServer::UdpServer()
{
    // Create the UDP socket
    // SOCK_STREAM for TCP / SOCK_DGRAM for UDP
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket\n");
    }

    // Bind the socket to any valid IP address and a specific port
    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
            perror("bind failed");
    }

    // Allocate the buffer
    // buffer = calloc(1, sizeof(struct MsgUdp));
}

int anloro::UdpServer::Receive(void * buffer)
{
    int recvlen;
    recvlen = recvfrom(fd, buffer, sizeof(MsgUdp), 0, (struct sockaddr *)&remaddr, &addrlen);
        
    return recvlen;
}

int anloro::UdpServer::Send(MsgUdp msg)
{
    if (sendto(fd, (char*)&msg, sizeof(MsgUdp), 0, (struct sockaddr *)&remaddr, slen)==-1){
        perror("sendto");
        return -1;
    }
        
    return 0;
}
