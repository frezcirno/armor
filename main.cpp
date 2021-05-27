//
// Created by sp on 2020-02-03
//
//

#include "attack.hpp"
#include "capture/capture.hpp"
#include "communicator.hpp"
#include "imageshow.hpp"
#include <windmill/Windmill.hpp>

int main() {
    using namespace std::chrono_literals;

    std::cout << "[General] Using OpenCV " << CV_VERSION << std::endl;

    int threadNum = stConfig.get<int>("auto.thread-num");  //线程数初始化

    /* 通信 */
#ifdef USE_USB
    CommunicatorUSB communicator;
    communicator.open(0x0477, 0x5620);
#else
    CommunicatorSerial communicator;
    communicator.disable(!stConfig.get<bool>("communicator.enable"));
    communicator.open(stConfig.get<std::string>("communicator.serial-port"));
#endif
    communicator.startReceiveService();  // 开线程

    /* 开摄像头或视频 */
    Capture *cap = nullptr;

    if (stConfig.get<bool>("cap.is-video")) {  //打开视频或者垃圾摄像头
        cap = new LajiVision();
    } else {
#ifdef MINDVISION
        cap = new MindVision();
#elif DAHENG
        cap = new DahengVision();
#else
        cap = new DaHuaVision();
#endif
    }

    cap->init();  // 初始化

    if (stConfig.get<bool>("cap.is-video")) {
        cap->setCaptureROI(cv::Size2i(1280, 900), CAP_ROI_CENTER_CENTER);  // 采集ROI
    } else {
        cap->setCaptureROI(cv::Size2i(1280, 900), CAP_ROI_CENTER_CENTER);  // 采集ROI
    }
    cap->play();  // 开始采集图像

    /* 开图像显示辅助程序 */
    ImageShowServer isServer(threadNum, 0.5);
    isServer.setMode(stConfig.get<int>("isServer.mode"));  //配置模式，三种，具体看配置文件
    isServer.setFontSize(1.25);
    isServer.enableClockPrint(true);
    isServer.enableAverageCostPrint(true);

    /* attack线程组 */
    std::vector<std::thread> attackThreads;
    attackThreads.resize(threadNum);

    /* PID算法 */
    PID pid;
    pid.init(stConfig.get<double>("auto.kp"),
        stConfig.get<double>("auto.ki"),
        stConfig.get<double>("auto.kd"));

    /* 图像旋转角度（适配摄像头装反的情况） */
    int rotate = -1;
    switch (stConfig.get<int>("cap.rotate")) {
        case 0: rotate = -1; break;
        case 90: rotate = cv::ROTATE_90_CLOCKWISE; break;
        case 180: rotate = cv::ROTATE_180; break;
        case 270: rotate = cv::ROTATE_90_COUNTERCLOCKWISE; break;
        default: PRINT_WARN("[config] cap.rotate: invalid value"); break;
    }

    for (int i = 0; i < threadNum; ++i) {
        /**
         * 每个线程都会对attack、windmill进行初始化，之后会根据循环中i的值来为不同到线程分配不同到击打任务（自瞄或者风车击打）
         * 作为std::thread参数的匿名函数是线程的执行体
         */
        attackThreads[i] = std::thread([cap, &isServer, i, &communicator, &pid, rotate]() {
            ImageShowClient isClient = isServer.getClient(i);

            /* 初始化 attack */
            Attack attack(communicator, pid, isClient);
            attack.enablePredict(stConfig.get<bool>("auto.enable-predict"));            //是否进行预测
            attack.setMode(stConfig.get<std::string>("attack.attack-color") == "red");  //击打颜色为红色，具体见配置文件

            /* 初始化 windmill */
            cv::Mat TvCtoL = (cv::Mat_<double>(3, 1) << stConfig.get<double>("power.x"), stConfig.get<double>("power.y"), stConfig.get<double>("power.z"));  //摄像头到云台转化矩阵

            double delay = stConfig.get<double>("power.delay");
            double maxPitchError = stConfig.get<double>("power.maxPitchError");
            double maxYawError = stConfig.get<double>("power.maxYawError");

            /* 风车实例化 */
            wm::Windmill *pWindMill =
                wm::Windmill::GetInstance(stCamera.camMat, stCamera.distCoeffs, TvCtoL, delay, "../rmdl/models/symbol.onnx", &isClient, maxPitchError, maxYawError);
            /*pWindMill->thre = tn::stupid::fixed_thre = tn::windmill::fixed_thre = stConfig.get<int>("power.thre");
            pWindMill->close = stConfig.get<bool>("power.close");
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
                    if (rotate != -1) {
                        cv::rotate(frame, frame, rotate);
                    }
                    /* 刷新主线程窗口图像 */
                    isClient.update(frame, int(timeStamp / 1000));
                    isClient.addText(cv::format("size %d x %d", stFrameInfo.size.width, stFrameInfo.size.height));
                    isClient.addText(cv::format("ts %ld", timeStamp));
                    isClient.addText(cv::format("fps %3ld", 1000000 / cap->getCurrentInterval()));
                    isClient.addText(cv::format("send %2.2f ms", communicator.getCurrentInterval() / 1000.0));

                    isClient.clock("run");

                    /* 获取云台角度 */
                    communicator.getGlobalAngle(&gYaw, &gPitch);

                    /* 获取工作模式(风车/装甲板) */
                    emWorkMode mode = communicator.getWorkMode();

                    switch (mode) {
                        case RM_AUTO_ATTACK:  //跑自瞄
                            isClient.addText("mode: RM_AUTO_ATTACK");
                            if (attack.run(frame, timeStamp, gYaw, gPitch))
                                /* 通知主线程显示图像, 有时候这一帧放弃的话就不显示了 */
                                isClient.show();
                            break;
                        case RM_WINDMILL_SMALL_CLOCK:
                        case RM_WINDMILL_SMALL_ANTIC:
                        case RM_WINDMILL_LARGE_CLOCK:
                        case RM_WINDMILL_LARGE_ANTIC:  // 跑风车
                            float pitch = 0.0;
                            float yaw = 0.0;
                            switch (mode) {  //选择模式初始化参数（这是旧版，最新版未更新）
                                case RM_WINDMILL_SMALL_CLOCK:
                                    isClient.addText("mode: RM_WINDMILL_SMALL_CLOCK");
                                    pWindMill->delay = delay;
                                    pWindMill->maxPitchError = maxPitchError;
                                    pWindMill->maxYawError = maxYawError;
                                    pWindMill->mode = "linear";
                                    break;
                                case RM_WINDMILL_SMALL_ANTIC:
                                    isClient.addText("mode: RM_WINDMILL_SMALL_ANTIC");
                                    pWindMill->delay = -delay;
                                    pWindMill->maxPitchError = -maxPitchError;
                                    pWindMill->maxYawError = -maxYawError;
                                    pWindMill->mode = "linear";
                                    break;
                                case RM_WINDMILL_LARGE_CLOCK:
                                    isClient.addText("mode: RM_WINDMILL_CLOCK");
                                    pWindMill->maxPitchError = maxPitchError;
                                    pWindMill->maxYawError = maxYawError;
                                    pWindMill->mode = "triangular";
                                    break;
                                case RM_WINDMILL_LARGE_ANTIC:
                                    isClient.addText("mode: RM_WINDMILL_ANTIC");
                                    pWindMill->maxPitchError = -maxPitchError;
                                    pWindMill->maxYawError = -maxYawError;
                                    pWindMill->mode = "triangular";
                                    break;
                            }

                            if (pWindMill->run(frame, pitch, yaw, (double)cv::getTickCount())) {  //有目标，运行风车击打
                                communicator.send(0.0, 0.0, SEND_STATUS_WM_AIM, SEND_STATUS_WM_FIND);
                                isClient.addText(cv::format("send pitch:%0.2f", pitch));
                                isClient.addText(cv::format("send yaw:%0.2f", yaw));

                            } else {  //无目标
                                communicator.send(0.0, 0.0,
                                    SEND_STATUS_WM_AIM, SEND_STATUS_WM_NO);
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
                    std::this_thread::sleep_for(5us);
                }
            }
            PRINT_INFO("attackThreads %d quit \n", i);
        });
    }

    // 图像显示主循环
    isServer.mainloop();

    for (auto &_t : attackThreads)
        _t.join();

#ifndef USE_USB
    communicator.letStop();
    communicator.join();
#endif

    return 0;
}
