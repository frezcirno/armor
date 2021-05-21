//
// Created by sp on 19-6-25.
//

#include "communicator.hpp"


int main() {
//    CommunicatorUSB communicator;
//    communicator.open(0x0477, 0x5620);

    CommunicatorSerial communicator;
    communicator.open("/dev/ttyUSB0");

    communicator.enableReceiveGlobalAngle(true);

    communicator.startReceiveService();

    while (true) {
        communicator.send(5, 0, SEND_STATUS_AUTO_AIM, SEND_STATUS_WM_PLACEHOLDER);
    }

    communicator.join();
    printf("main end\n");
    return 0;
}