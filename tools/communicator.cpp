//
// Created by sp on 19-6-25.
//

#include "communicator.hpp"


int main() {
//    armor::CommunicatorUSB communicator;
//    communicator.open(0x0477, 0x5620);

    armor::CommunicatorSerial communicator;
    communicator.open("/dev/USBsp2");

    communicator.enableReceiveGlobalAngle(true);

    communicator.startReceiveService();

    while (true) {
        communicator.send(5, 0, armor::SEND_STATUS_AUTO_AIM, armor::SEND_STATUS_WM_PLACEHOLDER);
    }

    communicator.join();
    printf("main end\n");
    return 0;
}