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
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    int init() {
        // Creating socket file descriptor
        if ((listenSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("socket failed");
            return -1;
        }

        int opt = 1;
        if (setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            perror("setsockopt");
            return -1;
        }

        return 0;
    }

  public:
    DebugSocket(unsigned short port = 8880) {
        init();

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr("127.0.0.1");
        address.sin_port = htons(port);
    }

    ~DebugSocket() {
        close(listenSock);
    }

    void sendFrame(const cv::Mat &frame) const {
        std::vector<unsigned char> sendBuf;
        cv::imencode(".jpg", frame, sendBuf);
        std::string str_encode(sendBuf.begin(), sendBuf.end());
        size_t size = str_encode.size();
        PRINT_INFO("[DEBUG] Send %d bytes\n", sendto(listenSock, &size, sizeof(size), 0, (struct sockaddr *)&address, addrlen));
        PRINT_INFO("[DEBUG] Send %d bytes\n", sendto(listenSock, str_encode.c_str(), size, 0, (struct sockaddr *)&address, addrlen));
    }
};