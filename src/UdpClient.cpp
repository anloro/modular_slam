// Client side implementation of UDP client-server model
#include "UdpClientServer.h"

using namespace anloro;


anloro::UdpClient::UdpClient()
{
    // Create the UDP socket
    // SOCK_STREAM for TCP / SOCK_DGRAM for UDP
    if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1) {
        printf("socket created\n");
    }

    // Bind the socket to any valid IP address and a specific port
    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(0);

    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
            perror("bind failed");
    }

    /* now define remaddr, the address to whom we want to send messages */
    /* For convenience, the host address is expressed as a numeric IP address */
    /* that we will convert to a binary format via inet_aton */

    memset((char *) &remaddr, 0, sizeof(remaddr));
    remaddr.sin_family = AF_INET;
    remaddr.sin_port = htons(port);
    if (inet_aton(server, &remaddr.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
}

int anloro::UdpClient::Send(MsgUdp msg)
{
    if (sendto(fd, (char*)&msg, sizeof(MsgUdp), 0, (struct sockaddr *)&remaddr, slen)==-1){
        perror("sendto");
        return -1;
    }
        
    return 0;
}

int anloro::UdpClient::Receive(void * buffer)
{
    int recvlen;
    recvlen = recvfrom(fd, buffer, sizeof(MsgUdp), 0, (struct sockaddr *)&remaddr, &addrlen);
        
    return recvlen;
}

// int main(void)
// {
//     UdpClient client = UdpClient();
//     MsgUdp newMsg;
//     newMsg.type = 'a';
//     newMsg.element.refFrame.pose.x = 0.1;
//     newMsg.element.refFrame.pose.y = 2.0;
//     newMsg.element.refFrame.pose.z = 0.0;
//     newMsg.element.refFrame.pose.roll = 0.0;
//     newMsg.element.refFrame.pose.pitch = 0.0;
//     newMsg.element.refFrame.pose.yaw = 0.0;

//     void * buffer = calloc(1, sizeof(struct MsgUdp));

//     for (int i=0; i < 1000; i++) {
        
//         // Send the message
//         client.Send(newMsg);

//         // Receive the message from the client
//         int recvlen = client.Receive(buffer);
//         // Interpret the data (in this case we receive what we sent)
//         MsgUdp * msg = (struct MsgUdp *)buffer;

//         std::cout << msg->element.refFrame.pose.x << std::endl;
//         std::cout << msg->element.refFrame.pose.y << std::endl;
//         std::cout << msg->element.refFrame.pose.z << std::endl;
//         std::cout << msg->element.refFrame.pose.roll << std::endl;
//         std::cout << msg->element.refFrame.pose.pitch << std::endl;
//         std::cout << msg->element.refFrame.pose.yaw << std::endl;
//     }

//     return 0;
// }