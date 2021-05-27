// Server side C/C++ program to demonstrate Socket programming
#include "debug.h"
#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

class DebugSocket {
  private:
    int listenSock = -1;
    std::thread thread;
    struct sockaddr_in serv_addr;
    int addrlen = sizeof(serv_addr);

  public:
    DebugSocket(unsigned short port = 8880) {
        // Creating socket file descriptor
        if ((listenSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("socket failed");
        }

        int opt = 1;
        if (setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            perror("setsockopt");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        serv_addr.sin_port = htons(port);
    }

    ~DebugSocket() {
        close(listenSock);
    }

    void startUdpServerThread() {
        std::thread server = std::thread([&] {
            char recv_buf[20];
            struct sockaddr_in client_addr;
            int len;
            // while (true) {
            //     recvfrom(listenSock, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&client_addr, (socklen_t *)&len);
            //     while ()
            //     {
            //         sendto(listenSock, &size, sizeof(size), 0, (struct sockaddr *)&serv_addr, addrlen));
            //         sendto(listenSock, str_encode.c_str(), size, 0, (struct sockaddr *)&serv_addr, addrlen));
            //     }
            // }
        });
    }

    /**
     * 发送一帧图片
     */
    void sendFrame(const cv::Mat &frame) const {
        std::vector<unsigned char> sendBuf;
        cv::imencode(".jpg", frame, sendBuf);
        std::string str_encode(sendBuf.begin(), sendBuf.end());
        size_t size = str_encode.size();
        PRINT_INFO("[DEBUG] Send %ld bytes\n", sendto(listenSock, &size, sizeof(size), 0, (struct sockaddr *)&serv_addr, addrlen));
        PRINT_INFO("[DEBUG] Send %ld bytes\n", sendto(listenSock, str_encode.c_str(), size, 0, (struct sockaddr *)&serv_addr, addrlen));
    }
};