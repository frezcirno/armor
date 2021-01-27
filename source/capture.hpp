//
// Created by sp on 19-6-15.
// 摄像头
//

#ifndef ATTACK_CAPTURE_HPP
#define ATTACK_CAPTURE_HPP

#include <atomic>
#include <base.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "video.hpp"    // dahua
#include <CameraApi.h>  // mindvision
//#include <MvCameraControl.h>  // hikvision

namespace armor {
/**
 * 采集时ROI预设位置
 */
typedef enum {
    CAP_ROI_TOP_LEFT,
    CAP_ROI_TOP_CENTER,
    CAP_ROI_TOP_RIGHT,
    CAP_ROI_CENTER_LEFT,
    CAP_ROI_CENTER_CENTER,
    CAP_ROI_CENTER_RIGHT,
    CAP_ROI_BOTTOM_LEFT,
    CAP_ROI_BOTTOM_CENTER,
    CAP_ROI_BOTTOM_RIGHT
} emCapROIPosition;

/**
 * 采集基类
 */
class Capture {
  protected:
    std::mutex m_mutex;
    std::atomic<int64> m_currentInterval;  // us

    /* 录视频相关 */
    cv::VideoWriter m_writer;
    bool m_isEnableRecord;
    std::thread m_recordThread;
    Semaphore m_semaphore;
    cv::Mat m_recordFrame;

    /**
     * 录的视频文件名数字自增
     * @param filename
     * @return
     */
    static std::string m_genVideoName(const std::string &filename) {
        int name = 0;
        std::ifstream ifs("../data/video/" + filename);
        ifs >> name;
        ifs.close();

        name++;
        std::ofstream ofs("../data/video/" + filename);
        ofs << name;
        ofs.close();
        return cv::format("%d", name);
    }

  public:
    explicit Capture() : m_currentInterval(0), m_isEnableRecord(false) {}

    /**
     * 初始化摄像头
     * @return
     */
    virtual bool init() {}

    /**
     * 摄像头开始传输图像
     * @return
     */
    virtual bool play() {}

    /**
     * 初始化图像数据结构
     * @param frame
     */
    virtual void initFrameMat(cv::Mat &frame) {}

    /**
     * 开启ROI, 工业相机是设置的硬件ROI
     * @param size ROI大小
     * @param emPosition 预设的位置, 见 emCapROIPosition
     */
    void setCaptureROI(const cv::Size2i &size, emCapROIPosition emPosition) {
        cv::Point2i offset = cv::Point2i(0, 0);
        switch (emPosition) {
            case CAP_ROI_BOTTOM_LEFT:
                offset.y = stFrameInfo.size.height - size.height;
                offset.x = 0;
                break;
            case CAP_ROI_BOTTOM_CENTER:
                offset.y = stFrameInfo.size.height - size.height;
                offset.x = (stFrameInfo.size.width - size.width) / 2;
                break;
            case CAP_ROI_BOTTOM_RIGHT:
                offset.y = stFrameInfo.size.height - size.height;
                offset.x = stFrameInfo.size.width - size.width;
                break;
            case CAP_ROI_TOP_LEFT:
                break;
            case CAP_ROI_TOP_CENTER:
                offset.x = (stFrameInfo.size.width - size.width) / 2;
                break;
            case CAP_ROI_TOP_RIGHT:
                offset.x = stFrameInfo.size.width - size.width;
                break;
            case CAP_ROI_CENTER_LEFT:
                offset.y = (stFrameInfo.size.height - size.height) / 2;
                offset.x = 0;
                break;
            case CAP_ROI_CENTER_CENTER:
                offset.y = (stFrameInfo.size.height - size.height) / 2;
                offset.x = (stFrameInfo.size.width - size.width) / 2;
                break;
            case CAP_ROI_CENTER_RIGHT:
                offset.y = (stFrameInfo.size.height - size.height) / 2;
                offset.x = stFrameInfo.size.width - size.width;
                break;
            default:
                break;
        }

        if (offset.x + size.width > stFrameInfo.size.width ||
            offset.y + size.height > stFrameInfo.size.height) {
            perror("[Capture] ROI params invalid\n");
            exit(-1);
        }
        setCaptureROI(size, offset);
    }

    /**
     * [重载函数]开启ROI, 工业相机是设置的硬件ROI
     * @param size ROI大小
     * @param offset 偏移量
     */
    virtual void setCaptureROI(const cv::Size2i &size,
        const cv::Point2i &offset) {}

    /**
     * 查询是否打开摄像头
     * @return
     */
    virtual bool isOpened() {}

    /**
     * 阻塞获取摄像头, 多线程可用
     * @param frame
     * @param timeStamp 时间戳 us
     * @param func
     * @return 是否有效
     */
    virtual bool wait_and_get(cv::Mat &frame, int64 &timeStamp, const std::function<void()> &func) {}

    /**
     * 获得当前 wait_and_get 函数获得的图像时间间隔
     * @return 采集时间间隔us
     */
    int64 getCurrentInterval() { return m_currentInterval.load(); }

    /**
     * 释放摄像头, 析构时自动调用
     */
    virtual void release() {}
};

/**
 * 辣鸡摄像头
 */
class LajiVision : public Capture {
  private:
    cv::VideoCapture m_cap;
    int64 m_startTickCount;
    int64 m_currentTimeStamp;

  public:
    explicit LajiVision()
        : m_startTickCount(cv::getTickCount()),
          m_currentTimeStamp(cv::getTickCount()){};

    /**
     * 初始化摄像头
     * @return
     */
    bool init() override {
        // 处理是不是要打开给一个视频
        const std::string filename = stConfig.get<std::string>("cap.video-path");
        if (filename.find(".avi") != cv::String::npos ||
            filename.find(".mp4") != cv::String::npos) {
            PRINT_INFO("open a video\n");
            m_cap.open(filename, cv::CAP_FFMPEG);
        } else {
            PRINT_INFO("open the laji camera\n");
            m_cap.open(filename, cv::CAP_V4L2);
            m_cap.set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            m_cap.set(cv::CAP_PROP_FPS, 120.0);
            m_cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
            m_cap.set(cv::CAP_PROP_EXPOSURE,
                stConfig.get<double>("auto.laji-exposure"));
            m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        }
        stFrameInfo.size.width = 1280;
        stFrameInfo.size.height = 900;
        return true;
    }

    /**
     * 摄像头开始传输图像
     * @return
     */
    bool play() override { return true; }

    /**
     * 查询是否打开摄像头
     * @return
     */
    bool isOpened() override { return m_cap.isOpened(); }

    using Capture::setCaptureROI;

    /**
     * 开启ROI, 工业相机是设置的硬件ROI
     * @param size ROI大小
     * @param offset 偏移量
     */
    void setCaptureROI(const cv::Size2i &size,
        const cv::Point2i &offset) override {
        stFrameInfo.size = size;
        stFrameInfo.offset = offset;
    }

    /**
     * 阻塞获得图像, 线程安全
     * @param frame 图像
     * @param timeStamp 时间戳 us
     * @param func 获得图像时需要调用的函数
     * @return false = 获取失败
     */
    bool wait_and_get(cv::Mat &frame, int64 &timeStamp, const std::function<void()> &func) override {
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        m_cap >> frame;
        resize(frame, frame, cv::Size(1920, 1280));

        // frame = frame(cv::Rect(stFrameInfo.offset, stFrameInfo.size));
        func();
        timeStamp = int64(1000.0 * 1000.0 * (cv::getTickCount() - m_startTickCount) / cv::getTickFrequency());
        m_currentInterval.exchange(timeStamp - m_currentTimeStamp);
        m_currentTimeStamp = timeStamp;
        return frame.data != nullptr;
    }

    /**
     * 释放摄像头, 析构时自动调用
     */
    void release() override { m_cap.release(); }

    ~LajiVision() { release(); }
};

#ifdef MINDVISION
/**
 * 迈德威视摄像头
 */
class MindVision : public Capture {
  private:
    int m_hCamera;  // 设备句柄
    BYTE *m_pbyBuffer;
    unsigned char *m_pRgbBuffer;
    std::atomic_bool m_isOpened;
    int64 m_currentTimeStamp;  // us

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
            armor::stConfig.get<double>("auto.pro-exposure"));  // 设置曝光时间(微秒)
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
        } else
            printf("[MindVision] EnumerateDevice = %d, Device count = %d\n", iStatus,
                iCameraCounts);

        /* 相机初始化. 初始化成功后, 才能调用任何其他相机相关的操作接口 */
        iStatus = CameraInit(&tCameraEnumList, -1, -1, &m_hCamera);
        /* 初始化失败处理 */
        if (iStatus != CAMERA_STATUS_SUCCESS) {
            perror("[MindVision] Camera Init failed");
            return false;
        } else
            printf("[MindVision] Camera Init state = %d\n", iStatus);

        /* 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
     */
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
        /* 让SDK进入工作模式, 开始接收来自相机发送的图像数据.
     * 如果当前相机是触发模式, 则需要接收到触发帧以后才会更新图像 */
        int iState = CameraPlay(m_hCamera);
        if (iState == CAMERA_STATUS_SUCCESS) {
            m_isOpened.exchange(true);
            CameraClearBuffer(m_hCamera);
            CameraReleaseImageBuffer(m_hCamera, m_pbyBuffer);
            printf("[MindVision] Camera Start Play\n");
            /*    处理录视频    */
            if (armor::stConfig.get<bool>("cap.record")) {
                std::string rC = "MJPG";
                int recordCode = cv::VideoWriter::fourcc(rC[0], rC[1], rC[2], rC[3]);
                std::string path = "../data/video/" + m_genVideoName("auto") + ".avi";
                m_writer.open(path, recordCode, 210, stFrameInfo.size);
                printf("| record: %s |\n", path.c_str());
                m_isEnableRecord = true;

                m_recordThread = std::thread([&]() {
                    while (m_isOpened) {
                        m_semaphore.wait_for(2s, [&]() { m_writer << m_recordFrame; });
                        thread_sleep_us(100);
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
    bool wait_and_get(cv::Mat &frame, int64 &timeStamp, const std::function<void()> &func) override {
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
            int64 t = cv::getTickCount();
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
                m_semaphore.signal_try([&]() { frame.copyTo(m_recordFrame); });
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

#elif DAHENG

#else
/**
 * 华睿科技摄像头
 */
class DaHuaVision : public Capture {
  private:
    ICameraPtr cameraSptr;         // 相机的句柄
    CFrame m_frame;                // 相机的块数据帧
    IStreamSourcePtr streamPtr;    // 流对象
    std::atomic_bool m_isOpened;   // 相机是否打开
    int64 m_currentTimeStamp = 0;  // us

  public:
    explicit DaHuaVision() : m_isOpened(false), m_currentTimeStamp(0){};

    /**
     * 设置相机参数
     */
    void m_setParams() {
        /* 设置相机为连续拉流模式 */
        setGrabMode(cameraSptr, true);
        // setGrabMode(cameraSptr, false);
        // bool bContious = false;
        // getGrabMode(cameraSptr, bContious);
        // triggerSoftware(cameraSptr);

        // int64_t nWidth, nHeight;
        // setResolution(cameraSptr, 1280, 1024);
        // getResolution(cameraSptr, nWidth, nHeight);
        // std::cout << "getResolution: " << nWidth << " " << nHeight << endl;

        // setBinning(cameraSptr);

        // getMaxResolution(cameraSptr, nWidth, nHeight);
        // cout << "getMaxResolution: " << nWidth << " " << nHeight << endl;

        // int64_t nX, nY, nROIWidth, nROIHeight;
        // setROI(cameraSptr, 0, 0, 1280, 1024);
        // getROI(cameraSptr, nX, nY, nROIWidth, nROIHeight);
        // std::cout << "getROI: " << nX << " " << nY << " " << nROIWidth << " " <<
        // nROIHeight << endl;

        // getWidth(cameraSptr, nWidth);
        // getHeight(cameraSptr, nHeight);

        double dExposureTime = 0;
        setExposureTime(cameraSptr,
            armor::stConfig.get<double>("auto.pro-exposure"), true);
        // setExposureTime(cameraSptr, 10000, false);
        // getExposureTime(cameraSptr, dExposureTime);
        // cout << "getExposureTime: " << dExposureTime << endl;

        // double dMinExposure, dMaxExposure;
        // getExposureTimeMinMaxValue(cameraSptr, dMinExposure, dMaxExposure);

        double dGainRaw = 0;
        double dGainRawMin = 0;
        double dGainRawMax = 0;
        setGainRaw(cameraSptr, 1.5);
        // getGainRaw(cameraSptr, dGainRaw);
        // getGainRawMinMaxValue(cameraSptr, dGainRawMin, dGainRawMax);
        // cout << "getGainRaw: " << dGainRaw << endl;

        double dGamma = 0;
        double dGammaMin = 0;
        double dGammaMax = 0;
        setGamma(cameraSptr, 0.8);
        // getGamma(cameraSptr, dGamma);
        // getGammaMinMaxValue(cameraSptr, dGammaMin, dGammaMax);

        double dRedBalanceRatio = 0;
        double dGreenBalanceRatio = 0;
        double dBlueBalanceRatio = 0;
        double dMinBalanceRatio = 0;
        double dMaxBalanceRatio = 0;
        setBalanceRatio(cameraSptr, 1.5, 1.5, 1.5);
        // getBalanceRatio(cameraSptr, dRedBalanceRatio, dGreenBalanceRatio,
        // dBlueBalanceRatio);
        // getBalanceRatioMinMaxValue(cameraSptr, dMinBalanceRatio,
        // dMaxBalanceRatio);

        double dFrameRate = 0;
        // setAcquisitionFrameRate(cameraSptr, 20);
        getAcquisitionFrameRate(cameraSptr, dFrameRate);
        std::cout << "getAcquisitionFrameRate: " << dFrameRate << std::endl;

        // userSetSave(cameraSptr);
        // loadUserSet(cameraSptr);

        // double dDelayTime = 0;
        // setTriggerDelay(cameraSptr, 20);
        // getTriggerDelay(cameraSptr, dDelayTime);

        // bool bRisingEdge = true;
        // setLineTriggerMode(cameraSptr, bRisingEdge);
        // getLineTriggerMode(cameraSptr, bRisingEdge);

        // double dLineDebouncerTimeAbs = 0;
        // setLineDebouncerTimeAbs(cameraSptr, 20);
        // getLineDebouncerTimeAbs(cameraSptr, dLineDebouncerTimeAbs);

        // setOutputTime(cameraSptr, 1000);

        // setReverseX(cameraSptr, false);
        // setReverseY(cameraSptr, false);
    }

    bool init() override {
        /* 发现设备 */
        CSystem &systemObj = CSystem::getInstance();
        TVector<ICameraPtr> vCameraPtrList;

        bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);

        if (!isDiscoverySuccess) {
            printf("discovery device fail.\n");
            return 0;
        }

        if (vCameraPtrList.size() == 0) {
            printf("no devices.\n");
            return 0;
        }

        // print camera info (index,Type,vendor name, model,serial
        // number,DeviceUserID,IP Address)
        // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
        displayDeviceInfo(vCameraPtrList);
        // int cameraIndex = selectDevice(vCameraPtrList.size());

        // cameraSptr = vCameraPtrList[cameraIndex];
        cameraSptr = vCameraPtrList[0];
        /* GigE相机时，连接前设置相机Ip与网卡处于同一网段上 */
        if (ICamera::typeGige == cameraSptr->getType()) {
            if (autoSetCameraIP(cameraSptr) != 0) {
                printf("set camera Ip failed.\n");
            }
        }

        /* 连接相机 */
        if (!cameraSptr->connect()) {
            printf("connect cameral failed.\n");
            return 0;
        }

        m_setParams();

        /* ROI初始化 */
        stFrameInfo.size = cv::Size(1280, 1024);
        stFrameInfo.offset = cv::Point2i(0, 0);

        /* 创建流对象 */
        streamPtr = systemObj.createStreamSource(cameraSptr);
        if (NULL == streamPtr) {
            printf("create stream obj  fail.\r\n");
            return 0;
        }

        printf("[DaHua] complete init\n");
    }

    /**
     * [重载函数]开启ROI, 工业相机是设置的硬件ROI
     * @param size ROI大小
     * @param offset 偏移量
     */
    void setCaptureROI(const cv::Size2i &size,
        const cv::Point2i &offset) override {
        /* 硬件ROI */
        setResolution(cameraSptr, size.width, size.height);
        setROI(cameraSptr, offset.x, offset.y, size.width, size.height);

        stFrameInfo.size = size;
        stFrameInfo.offset = offset;
        printf("[DaHua] setCaptureROI end\n");
    }

    /**
     * 开始采集图像
     * @return true / false
     */

    bool play() override {
        /* 让SDK进入工作模式, 开始接收来自相机发送的图像数据.
     * 如果当前相机是触发模式, 则需要接收到触发帧以后才会更新图像 */
        /* 开始取图 */
        bool isStartGrabbingSuccess = streamPtr->startGrabbing();
        if (isStartGrabbingSuccess) {
            m_isOpened.exchange(true);
            // m_frame.reset();
            printf("[DaHua] Camera Start Play\n");
            /*    处理录视频    */
            if (armor::stConfig.get<bool>("cap.record")) {
                std::string rC = "MJPG";
                int recordCode = cv::VideoWriter::fourcc(rC[0], rC[1], rC[2], rC[3]);
                std::string path = "../data/video/" + m_genVideoName("auto") + ".avi";
                m_writer.open(path, recordCode, 210, stFrameInfo.size);
                printf("| record: %s |\n", path.c_str());
                m_isEnableRecord = true;

                m_recordThread = std::thread([&]() {
                    while (m_isOpened) {
                        m_semaphore.wait_for(2s, [&]() { m_writer << m_recordFrame; });
                        thread_sleep_us(100);
                    }
                    PRINT_WARN("record quit\n");
                });
            }
            return true;
        } else {
            perror("[DaHua] Camera Play failed");
            printf("StartGrabbing fail.\n");
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
    bool wait_and_get(cv::Mat &frame, int64 &timeStamp, const std::function<void()> &func) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        UINT low;
        UINT high;
        streamPtr->getFrame(m_frame, 300);
        if (m_frame.valid()) {
            /* 获得图像时需要调用的自定义函数 */
            func();

            /* cvtColor图像转换, sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_BAYGB8 */
            int64 t = cv::getTickCount();
            cv::Mat by8Mat(
                cv::Size(m_frame.getImageWidth(), m_frame.getImageHeight()), CV_8UC1,
                (uchar *)m_frame.getImage());
            cv::cvtColor(by8Mat, frame, cv::COLOR_BayerGB2RGB_EA);

            //                /* sdk函数图像转换 */
            //                CameraImageProcess(m_hCamera, m_pbyBuffer, m_pRgbBuffer,
            //                &sFrameInfo);
            //                /* 刷新图像 */
            //                frame.data = m_pRgbBuffer;

            printf("[DaHua] convert rgb cost: %.2f ms\n",
                1000 * (cv::getTickCount() - t) / cv::getTickFrequency());

            if (m_isEnableRecord) {
                m_semaphore.signal_try([&]() { frame.copyTo(m_recordFrame); });
            }

            /* 获得时间戳 */
            timeStamp = m_frame.getImageTimeStamp();
            m_currentInterval.exchange(timeStamp - m_currentTimeStamp);
            m_currentTimeStamp = timeStamp;

            /* 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
       * 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
       */
            m_frame.reset();
            return true;
        } else {
            return false;
        }
    }
};
#endif

}  // namespace armor

#endif  // ATTACK_CAPTURE_HPP
