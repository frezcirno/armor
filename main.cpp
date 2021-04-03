//
// Created by sp on 2020-02-03
//
//

#include "attack.hpp"
#include "capture.hpp"
#include "communicator.hpp"
#include "imageshow.hpp"
#include <windmill/Windmill.hpp>

int main() {
    std::cout << "[General] Using OpenCV " << CV_VERSION << std::endl;

    int threadNum = armor::stConfig.get<int>("auto.thread-num");  //线程数初始化

    /* 通信 */
#ifdef USE_USB
    armor::CommunicatorUSB communicator;
    communicator.open(0x0477, 0x5620);
#else
    armor::CommunicatorSerial communicator;
    communicator.disable(!armor::stConfig.get<bool>("communicator.enable"));
    communicator.open(armor::stConfig.get<std::string>("communicator.serial-port"));
#endif
    communicator.startReceiveService();  // 开线程

    /* 开摄像头或视频 */
    armor::Capture *cap = nullptr;

    if (armor::stConfig.get<bool>("cap.is-video")) {  //打开视频或者垃圾摄像头
        cap = new armor::LajiVision();
    } else {
#ifdef MINDVISION
        cap = new armor::MindVision();

#elif DAHENG
        cap = new armor::DahengVision();

#else
        cap = new armor::DaHuaVision();

#endif
    }

    cap->init();  // 初始化

    if (armor::stConfig.get<bool>("cap.is-video")) {
        cap->setCaptureROI(cv::Size2i(1280, 900), armor::CAP_ROI_CENTER_CENTER);  // 采集ROI
    } else {
        cap->setCaptureROI(cv::Size2i(1280, 900), armor::CAP_ROI_CENTER_CENTER);  // 采集ROI
    }
    cap->play();  // 开始采集图像

    /* 开图像显示辅助程序 */
    armor::ImageShowServer isServer(threadNum, 0.5);
    isServer.setMode(armor::stConfig.get<int>("isServer.mode"));  //配置模式，三种，具体看配置文件
    isServer.setFontSize(1.25);
    isServer.enableClockPrint(true);
    isServer.enableAverageCostPrint(true);

    /* attack线程组 */
    std::vector<std::thread> attackThreads;
    attackThreads.resize(threadNum);

    /* PID算法 */
    armor::PID pid;
    pid.init(armor::stConfig.get<double>("auto.kp"),
        armor::stConfig.get<double>("auto.ki"),
        armor::stConfig.get<double>("auto.kd"));

    for (int i = 0; i < threadNum; ++i) {
        /**
         * 每个线程都会对attack、windmill进行初始化，之后会根据循环中i的值来为不同到线程分配不同到击打任务（自瞄或者风车击打）
         * 作为std::thread参数的匿名函数是线程的执行体
         */
        attackThreads[i] = std::thread([cap, &isServer, i, &communicator, &pid]() {
            armor::ImageShowClient isClient = isServer.getClient(i);

            /* 初始化 attack */
            armor::Attack attack(communicator, pid, isClient);
            attack.enablePredict(armor::stConfig.get<bool>("auto.enable-predict"));            //是否进行预测
            attack.setMode(armor::stConfig.get<std::string>("attack.attack-color") == "red");  //击打颜色为红色，具体见配置文件

            /* 初始化 windmill */
            cv::Mat TvCtoL = (cv::Mat_<double>(3, 1) << armor::stConfig.get<double>("power.x"), armor::stConfig.get<double>("power.y"), armor::stConfig.get<double>("power.z"));  //摄像头到云台转化矩阵

            double delay = armor::stConfig.get<double>("power.delay");
            double maxPitchError = armor::stConfig.get<double>("power.maxPitchError");
            double maxYawError = armor::stConfig.get<double>("power.maxYawError");

            /* 风车实例化 */
            wm::Windmill *pWindMill =
                wm::Windmill::GetInstance(armor::stCamera.camMat, armor::stCamera.distCoeffs, TvCtoL, delay, "../rmdl/models/symbol.onnx", &isClient, maxPitchError, maxYawError);
            /*pWindMill->thre = tn::stupid::fixed_thre = tn::windmill::fixed_thre = armor::stConfig.get<int>("power.thre");
            pWindMill->close = armor::stConfig.get<bool>("power.close");
            tn::init();*/

            int64_t timeStamp = 0;
            cv::Mat frame;

            cap->initFrameMat(frame);
            float gYaw = 0.0;
            float gPitch = 0.0;

            while (cap->isOpened() && !isServer.isWillExit()) {
                if (cap->wait_and_get(frame, timeStamp, [&communicator, &gYaw, &gPitch]() {
                    communicator.getGlobalAngle(&gYaw, &gPitch);
                })) {
                    /* 刷新主线程窗口图像 */
                    isClient.update(frame, int(timeStamp / 1000));
                    isClient.addText(cv::format("size %d x %d", armor::stFrameInfo.size.width, armor::stFrameInfo.size.height));
                    isClient.addText(cv::format("ts %ld", timeStamp));
                    isClient.addText(cv::format("fps %3d", int(1000000000 / cap->getCurrentInterval())));
                    isClient.addText(cv::format("send %2.2f ms", communicator.getCurrentInterval() / 1000.0));

                    isClient.clock("run");

                    /* 获取云台角度 */
                    // communicator.getGlobalAngle(&gYaw, &gPitch);

                    /* 获取工作模式(风车/装甲板) */
                    armor::emWorkMode mode = communicator.getWorkMode();

                    switch (mode) {
                        case armor::RM_AUTO_ATTACK:  //跑自瞄
                            isClient.addText("mode: RM_AUTO_ATTACK");
                            if (attack.run(frame, timeStamp, gYaw, gPitch))
                                /* 通知主线程显示图像, 有时候这一帧放弃的话就不显示了 */
                                isClient.show();
                            break;
                        case armor::RM_WINDMILL_SMALL_CLOCK:
                        case armor::RM_WINDMILL_SMALL_ANTIC:
                        case armor::RM_WINDMILL_LARGE_CLOCK:
                        case armor::RM_WINDMILL_LARGE_ANTIC:  // 跑风车
                            float pitch = 0.0;
                            float yaw = 0.0;
                            switch (mode) {  //选择模式初始化参数（这是旧版，最新版未更新）
                                case armor::RM_WINDMILL_SMALL_CLOCK:
                                    isClient.addText("mode: RM_WINDMILL_SMALL_CLOCK");
                                    pWindMill->delay = delay;
                                    pWindMill->maxPitchError = maxPitchError;
                                    pWindMill->maxYawError = maxYawError;
                                    pWindMill->mode = "linear";
                                    break;
                                case armor::RM_WINDMILL_SMALL_ANTIC:
                                    isClient.addText("mode: RM_WINDMILL_SMALL_ANTIC");
                                    pWindMill->delay = -delay;
                                    pWindMill->maxPitchError = -maxPitchError;
                                    pWindMill->maxYawError = -maxYawError;
                                    pWindMill->mode = "linear";
                                    break;
                                case armor::RM_WINDMILL_LARGE_CLOCK:
                                    isClient.addText("mode: RM_WINDMILL_CLOCK");
                                    pWindMill->maxPitchError = maxPitchError;
                                    pWindMill->maxYawError = maxYawError;
                                    pWindMill->mode = "triangular";
                                    break;
                                case armor::RM_WINDMILL_LARGE_ANTIC:
                                    isClient.addText("mode: RM_WINDMILL_ANTIC");
                                    pWindMill->maxPitchError = -maxPitchError;
                                    pWindMill->maxYawError = -maxYawError;
                                    pWindMill->mode = "triangular";
                                    break;
                            }

                            if (pWindMill->run(frame, pitch, yaw, (double)cv::getTickCount())) {  //有目标，运行风车击打
                                communicator.send(0.0, 0.0, armor::SEND_STATUS_WM_AIM, armor::SEND_STATUS_WM_FIND);
                                isClient.addText(cv::format("send pitch:%0.2f", pitch));
                                isClient.addText(cv::format("send yaw:%0.2f", yaw));

                            } else {  //无目标
                                communicator.send(0.0, 0.0,
                                    armor::SEND_STATUS_WM_AIM, armor::SEND_STATUS_WM_NO);
                                PRINT_WARN("[windmill] no target find\n");
                            }
                            isClient.addText(cv::format("delay: %0.2f", pWindMill->delay));
                            isClient.addText(cv::format("maxPitchError: %0.2f", pWindMill->maxPitchError));
                            isClient.addText(cv::format("maxYawError: %0.2f", pWindMill->maxYawError));
                            isClient.show();
                            break;
                    }

                    static std::chrono::high_resolution_clock::time_point beg, end;
                    static double now_max = 0;
                    end = std::chrono::high_resolution_clock::now();
                    auto cost = std::chrono::duration<double, std::milli>((end - beg)).count();
                    beg = std::chrono::high_resolution_clock::now();
                    if (cost < 10000)
                        now_max = std::max(now_max, cost);
                    std::cout << "@@@@@@@@@@@@@@@ " << cost << " ms\n";
                    std::cout << "@@@@@@@@@@@@@@@ NOW MAX -> " << now_max << " ms\n";
                    isClient.clock("run");

                } else {
                    PRINT_ERROR("capture wait_and_get() failed once\n");
                }

                while (isServer.isPause()) {
                    armor::thread_sleep_us(5);
                }
            }
            PRINT_INFO("attackThreads %d quit \n", i);
        });
    }

    // 图像显示主循环
    isServer.mainloop();

    for (auto &_t : attackThreads)
        _t.join();

//    /////
//    int64 timeStamp = 0;
//    cv::Mat frame;
//    std::cout << "Using OpenCV " << CV_VERSION << std::endl;
//
//    cap->initFrameMat(frame);
//    float gYaw = 0.0;
//    float gPitch = 0.0;
//
//    armor::ImageShowClient isClient = isServer.getClient(0);
//    std::cout << "Using OpenCV " << CV_VERSION << std::endl;
//
//    while ((cap->isOpened()) && !isServer.isWillExit())
//    {
//        if (cap->wait_and_get(frame, timeStamp, [&communicator, &gYaw, &gPitch]() {}))
//        {
//            /* 刷新主线程窗口图像 */
//            isClient.update(frame, int(timeStamp / 1000));
//            isClient.addText(cv::format("ts %lld", timeStamp));
//            isClient.addText(cv::format("1/fps %2.2f ms", cap->getCurrentInterval() / 1000.0));
//            isClient.addText(cv::format("send %2.2f ms", communicator.getCurrentInterval() / 1000.0));
//
//            isClient.clock("run");
//
//            auto mode = communicator.getWorkMode();
//            // auto mode =  armor::RM_WINDMILL_CLOCK;
//            isClient.addText(cv::format("mode: %x", int(mode)));
//            isClient.addImg("zz", frame);
//            // cv::imshow("vgsh",frame);
//            // if(cv::waitKey(1)=='q') break;
//
//            isClient.show();
//
//            static std::chrono::high_resolution_clock::time_point beg, end;
//            static double now_max = 0;
//            end = std::chrono::high_resolution_clock::now();
//            auto cost = std::chrono::duration<double, std::milli>((end - beg)).count();
//            beg = std::chrono::high_resolution_clock::now();
//            if (cost < 10000)
//                now_max = std::max(now_max, cost);
//            std::cout << "@@@@@@@@@@@@@@@ " << cost << " ms\n";
//            std::cout << "@@@@@@@@@@@@@@@ NOW MAX -> " << now_max << " ms\n";
//            isClient.clock("run");
//        }
//        else
//        {
//            PRINT_ERROR("capture wait_and_get() failed once\n");
//        }
//
//        while (isServer.isPause())
//        {
//            armor::thread_sleep_us(5);
//        }
//    }
//
//    // 图像显示主循环
//    isServer.mainloop();
///////
#ifndef USE_USB
    communicator.letStop();
    communicator.join();
#endif

    std::cout << "Attack End" << std::endl;
    return 0;
}
