#pragma once

#include <CameraApi.h>  // mindvision
#include <functional>

#include "base.hpp"
#include "capture/base_capture.hpp"
#include "debug.h"

/**
 * 迈德威视摄像头
 */
class MindVision : public Capture {
  private:
    int m_hCamera;  // 设备句柄
    BYTE *m_pbyBuffer;
    unsigned char *m_pRgbBuffer;
    std::atomic_bool m_isOpened;
    int64_t m_currentTimeStamp;  // us

    /**
     * 相机连接事件回调函数
     * @param hCamera
     * @param MSG
     * @param uParam
     * @param pContext
     */
    static void m_connectCallback(CameraHandle hCamera, UINT MSG, UINT uParam, PVOID pContext) {
        // 0: 相机连接断开    1: 相机连接恢复
        if (MSG == 0)
            printf("[MindVision] 相机连接断开, MSG %d \n", MSG);
        else
            printf("[MindVision] 相机连接恢复, MSG %d \n", MSG);
    }

    /**
     * 设置相机参数
     */
    void m_setParams() {
        /* 加载相机基础配置 */
        char a[] = "../data/MindVision.Config";
        CameraReadParameterFromFile(m_hCamera, a);

        /* 帧率 */
        CameraSetFrameSpeed(m_hCamera, FRAME_SPEED_LOW);

        /* 曝光 */
        CameraSetAeState(m_hCamera, FALSE);  // 设置手动曝光
        CameraSetExposureTime(
            m_hCamera,
            stConfig.get<double>("auto.pro-exposure"));  // 设置曝光时间(微秒)
        double iExposure = 0;
        CameraGetExposureTime(m_hCamera, &iExposure);
        printf("[MindVision] 曝光时间= %.f us\n", iExposure);

        /* 增益 */
        INT iAnalogGain;
        CameraSetAnalogGain(m_hCamera, 1);        // 模拟增益
        CameraSetGain(m_hCamera, 100, 100, 100);  // 数字增益
        int rG, gG, bG;
        CameraGetGain(m_hCamera, &rG, &gG, &bG);
        CameraGetAnalogGain(m_hCamera, &iAnalogGain);
        printf("[MindVision] 模拟增益: %d\n", iAnalogGain);
        printf("[MindVision] 数字增益: %d, %d, %d\n", rG, gG, bG);

        /* 降噪 */
        BOOL pEnable;
        CameraSetNoiseFilter(m_hCamera, TRUE);  // 使能降噪模块
        CameraGetNoiseFilterState(m_hCamera, &pEnable);
        std::cout << "[MindVision] 降噪模块 [使能=" << TRUE << "] " << pEnable
                  << std::endl;

        /* 白平衡 */
        CameraSetWbMode(m_hCamera, FALSE);  // 手动白平衡
        CameraGetWbMode(m_hCamera, &pEnable);
        CameraSetPresetClrTemp(m_hCamera, 1);
        std::cout << "[MindVision] 白平衡 [自动=" << TRUE << "] " << pEnable
                  << std::endl;

        /* 饱和度 */
        CameraSetSaturation(m_hCamera, 200);

        /* 锐利度 */
        CameraSetSharpness(m_hCamera, 100);

        /* 伽马 */
        CameraSetGamma(m_hCamera, 55);

        /* 覆盖保存相机配置 */
        //            char savePath[] = "../data/MindVision2.Config";
        //            CameraSaveParameterToFile(m_hCamera, savePath);
    }

  public:
    explicit MindVision()
        : m_hCamera(-1), m_pbyBuffer(nullptr), m_pRgbBuffer(nullptr),
          m_isOpened(false), m_currentTimeStamp(0){};

    bool init() override {
        printf("[MindVision] enter init\n");
        int iStatus;

        /* 相机SDK初始化，在调用任何SDK其他接口前 */
        CameraSdkInit(0);  // 0:表示英文, 1:表示中文

        /* 枚举设备，并建立设备列表 */
        tSdkCameraDevInfo tCameraEnumList;
        int iCameraCounts = 1;  // 需要枚举的设备数量
        iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
        /* 没有连接设备 处理 */
        if (iCameraCounts == 0 || iStatus != CAMERA_STATUS_SUCCESS) {
            perror("[MindVision] No Device Found");
            exit(-1);
        } else {
            printf("[MindVision] EnumerateDevice = %d, Device count = %d\n", iStatus,
                iCameraCounts);
        }

        /* 相机初始化. 初始化成功后, 才能调用任何其他相机相关的操作接口 */
        iStatus = CameraInit(&tCameraEnumList, -1, -1, &m_hCamera);
        /* 初始化失败处理 */
        if (iStatus != CAMERA_STATUS_SUCCESS) {
            perror("[MindVision] Camera Init failed");
            return false;
        } else {
            printf("[MindVision] Camera Init state = %d\n", iStatus);
        }

        /* 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数 */
        tSdkCameraCapbility tCapability;  // 设备描述信息
        CameraGetCapability(m_hCamera, &tCapability);
        m_pRgbBuffer =
            (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax *
                                    tCapability.sResolutionRange.iWidthMax * 3);

        /* 设置相机参数 */
        m_setParams();

        /* 掉线回调函数 */
        CameraSetConnectionStatusCallback(m_hCamera, &MindVision::m_connectCallback,
            nullptr);

        /* 黑白或彩色相机判断 */
        if (tCapability.sIspCapacity.bMonoSensor) {
            CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_MONO8);
        } else {
            CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_BGR8);
        }

        /* ROI初始化 */
        stFrameInfo.size = cv::Size(1280, 1024);
        stFrameInfo.offset = cv::Point2i(0, 0);
        printf("[MindVision] complete init\n");

        return true;
    }

    using Capture::setCaptureROI;

    /**
     * [重载函数]开启ROI, 工业相机是设置的硬件ROI
     * @param size ROI大小
     * @param offset 偏移量
     */
    void setCaptureROI(const cv::Size2i &size,
        const cv::Point2i &offset) override {
        /* 硬件ROI */
        tSdkImageResolution imageResolution;
        imageResolution.iIndex = 0xff;
        imageResolution.uBinSumMode = 0;
        imageResolution.uBinAverageMode = 0;
        imageResolution.uSkipMode = 0;
        imageResolution.uResampleMask = 0;
        imageResolution.iHOffsetFOV = offset.x;  // 特么SDK的注释是错的
        imageResolution.iVOffsetFOV = offset.y;
        imageResolution.iWidthFOV = size.width;
        imageResolution.iHeightFOV = size.height;
        imageResolution.iWidth = size.width;
        imageResolution.iHeight = size.height;
        imageResolution.iWidthZoomHd = 0;
        imageResolution.iHeightZoomHd = 0;
        imageResolution.iWidthZoomSw = 0;
        imageResolution.iHeightZoomSw = 0;
        CameraSetImageResolution(m_hCamera, &imageResolution);
        stFrameInfo.size = size;
        stFrameInfo.offset = offset;
        printf("[MindVision] setCaptureROI end\n");
    }

    /**
     * 开始采集图像
     * @return true / false
     */
    bool play() override {
        using namespace std::chrono_literals;
        /**
         * 让SDK进入工作模式, 开始接收来自相机发送的图像数据.
         * 如果当前相机是触发模式, 则需要接收到触发帧以后才会更新图像
         */
        int iState = CameraPlay(m_hCamera);
        if (iState == CAMERA_STATUS_SUCCESS) {
            m_isOpened.exchange(true);
            CameraClearBuffer(m_hCamera);
            CameraReleaseImageBuffer(m_hCamera, m_pbyBuffer);
            printf("[MindVision] Camera Start Play\n");
            /*    处理录视频    */
            if (m_isEnableRecord = stConfig.get<bool>("cap.record")) {
                std::string rC = "MJPG";
                int recordCode = cv::VideoWriter::fourcc(rC[0], rC[1], rC[2], rC[3]);
                std::string path = "../data/video/" + m_genVideoName("auto") + ".avi";
                m_writer.open(path, recordCode, 210, stFrameInfo.size);
                printf("| record: %s |\n", path.c_str());

                m_recordThread = std::thread([&] {
                    while (m_isOpened) {
                        {
                            std::unique_lock<std::mutex> lock(m_recordLock);
                            if (m_cond.wait_for(lock, 2s, [&] { return m_record.load(); })) {
                                m_writer << m_recordFrame;
                            }
                        }
                        std::this_thread::sleep_for(100us);
                    }
                    PRINT_WARN("record quit\n");
                });
            }
            return true;
        } else {
            perror("[MindVision] Camera Play failed");
            printf("[MindVision] Camera Play state = %d\n", iState);
            exit(-1);
        }
    }

    /**
     * 初始化 Mat 的结构
     * @param frame
     */
    void initFrameMat(cv::Mat &frame) override {
        frame = cv::Mat(stFrameInfo.size, CV_8UC3);
        frame = 0;
    }

    /**
     * 当前是否打开了设备
     * @return true / false
     */
    bool isOpened() override { return m_isOpened.load(); }

    /**
     * 阻塞获得图像, 线程安全
     * @param frame 图像
     * @param timeStamp 时间戳 us
     * @param func 获得图像时需要调用的自定义函数
     * @return false = 获取失败
     */
    bool wait_and_get(cv::Mat &frame, int64_t &timeStamp, const std::function<void()> &func) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        tSdkFrameHead sFrameInfo;
        UINT low;
        UINT high;
        int iState = 0;
        iState = CameraGetImageBuffer(m_hCamera, &sFrameInfo, &m_pbyBuffer, 5000);
        if (iState == CAMERA_STATUS_SUCCESS) {
            /* 获得图像时需要调用的自定义函数 */
            func();

            /* cvtColor图像转换, sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_BAYGB8 */
            int64_t t = cv::getTickCount();
            cv::Mat by8Mat(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8UC1,
                m_pbyBuffer);
            if (m_pbyBuffer == nullptr) {
                PRINT_ERROR("[MindVision] nullptr\n");
                CameraReleaseImageBuffer(m_hCamera, m_pbyBuffer);
                return false;
            }
            cv::cvtColor(by8Mat, frame, cv::COLOR_BayerGB2RGB_EA);

            //                /* sdk函数图像转换 */
            //    CameraImageProcess(m_hCamera, m_pbyBuffer, m_pRgbBuffer,
            //    &sFrameInfo);
            //                /* 刷新图像 */
            //                frame.data = m_pRgbBuffer;

            printf("[MindVision] convert rgb cost: %.2f ms\n",
                1000 * (cv::getTickCount() - t) / cv::getTickFrequency());

            if (m_isEnableRecord) {
                std::unique_lock<std::mutex> lock(m_recordLock, std::try_to_lock);
                if (lock.owns_lock()) {
                    frame.copyTo(m_recordFrame);
                    m_record.exchange(true);
                    lock.unlock();
                    m_cond.notify_one();
                }
            }

            /* 获得时间戳 */
            CameraGetFrameTimeStamp(m_hCamera, &low, &high);
            timeStamp = low + (high < 32);
            m_currentInterval.exchange(timeStamp - m_currentTimeStamp);
            m_currentTimeStamp = timeStamp;

            /* 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
       * 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
       */
            CameraReleaseImageBuffer(m_hCamera, m_pbyBuffer);
            return true;
        }
        PRINT_ERROR("[MindVision] iState = %d\n", iState);
        /* 重启摄像头 */
        CameraCommonCall(m_hCamera, "reset_device()", nullptr, 0);
        return false;
    }

    /**
     * 释放设备
     */
    void release() override {
        m_isOpened.exchange(false);

        CameraStop(m_hCamera);
        CameraUnInit(m_hCamera);

        /* 注意，先反初始化后再free */
        free(m_pRgbBuffer);
        m_pRgbBuffer = nullptr;

        m_writer.release();
        if (m_recordThread.joinable())
            m_recordThread.join();
    }

    ~MindVision() {
        if (m_isOpened.load())
            release();
    }
};