#pragma once

#include "capture/base_capture.hpp"
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion-null"
#include "capture/dahua.hpp"  // dahua
//#include <MvCameraControl.h>  // hikvision
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

typedef unsigned int UINT;

/**
 * 华睿科技摄像头
 */
class DaHuaVision : public Capture {
  private:
    ICameraPtr cameraSptr;           // 相机的句柄
    CFrame m_frame;                  // 相机的块数据帧
    IStreamSourcePtr streamPtr;      // 流对象
    std::atomic_bool m_isOpened;     // 相机是否打开
    int64_t m_currentTimeStamp = 0;  // us

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

        // int64_t_t nWidth, nHeight;
        // setResolution(cameraSptr, 1280, 1024);
        // getResolution(cameraSptr, nWidth, nHeight);
        // std::cout << "getResolution: " << nWidth << " " << nHeight << endl;

        // setBinning(cameraSptr);

        // getMaxResolution(cameraSptr, nWidth, nHeight);
        // cout << "getMaxResolution: " << nWidth << " " << nHeight << endl;

        // int64_t_t nX, nY, nROIWidth, nROIHeight;
        // setROI(cameraSptr, 0, 0, 1280, 1024);
        // getROI(cameraSptr, nX, nY, nROIWidth, nROIHeight);
        // std::cout << "getROI: " << nX << " " << nY << " " << nROIWidth << " " <<
        // nROIHeight << endl;

        // getWidth(cameraSptr, nWidth);
        // getHeight(cameraSptr, nHeight);

        double dExposureTime = 0;
        /**
         * 不要开自动曝光！
         **/
        setExposureTime(cameraSptr, 10 * stConfig.get<double>("auto.pro-exposure"), false);

        // setExposureTime(cameraSptr, 10000, false);
        // getExposureTime(cameraSptr, dExposureTime);
        // cout << "getExposureTime: " << dExposureTime << endl;

        // double dMinExposure, dMaxExposure;
        // getExposureTimeMinMaxValue(cameraSptr, dMinExposure, dMaxExposure);

        // double dGainRaw = 0;
        // double dGainRawMin = 0;
        // double dGainRawMax = 0;
        // setGainRaw(cameraSptr, 1.5);
        // getGainRaw(cameraSptr, dGainRaw);
        // getGainRawMinMaxValue(cameraSptr, dGainRawMin, dGainRawMax);
        // cout << "getGainRaw: " << dGainRaw << endl;

        // double dGamma = 0;
        // double dGammaMin = 0;
        // double dGammaMax = 0;
        setGamma(cameraSptr, 0.8);
        // getGamma(cameraSptr, dGamma);
        // getGammaMinMaxValue(cameraSptr, dGammaMin, dGammaMax);

        // double dRedBalanceRatio = 0;
        // double dGreenBalanceRatio = 0;
        // double dBlueBalanceRatio = 0;
        // double dMinBalanceRatio = 0;
        // double dMaxBalanceRatio = 0;
        setBalanceRatio(cameraSptr, 1.49, 1.0, 1.0);
        // getBalanceRatio(cameraSptr, dRedBalanceRatio, dGreenBalanceRatio,
        // dBlueBalanceRatio);
        // getBalanceRatioMinMaxValue(cameraSptr, dMinBalanceRatio,
        // dMaxBalanceRatio);

        double dFrameRate = 0;
        // setAcquisitionFrameRate(cameraSptr, 20);
        getAcquisitionFrameRate(cameraSptr, dFrameRate);
        // std::cout << "getAcquisitionFrameRate: " << dFrameRate << std::endl;

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

        for (auto &&cam : vCameraPtrList) {
            auto venderName = std::string(cam->getVendorName());
            if (venderName.find("Dahua") != -1) {
                cameraSptr = cam;
                break;
            }
        }

        if (cameraSptr.get() == NULL) {
            PRINT_WARN("[Dahua] Use 0 camera");
            cameraSptr = vCameraPtrList[0];
        }

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

        /**
         * Dahua Camera Note: 
         * Max Frame Size = 1280 x 962 
         * We use the 1280 x 960 of it, So
         *  size = 1280, 960 
         *  offset = 0, 0
         */

        /* ROI初始化 */
        stFrameInfo.size = cv::Size(1280, 960);
        stFrameInfo.offset = cv::Point2i(0, 0);

        /* 创建流对象 */
        streamPtr = systemObj.createStreamSource(cameraSptr);
        if (NULL == streamPtr) {
            printf("create stream obj  fail.\r\n");
            return 0;
        }

        printf("[DaHua] complete init\n");

        return true;
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
        using namespace std::chrono_literals;
        /* 让SDK进入工作模式, 开始接收来自相机发送的图像数据.
         * 如果当前相机是触发模式, 则需要接收到触发帧以后才会更新图像 */
        /* 开始取图 */
        bool isStartGrabbingSuccess = streamPtr->startGrabbing();
        if (isStartGrabbingSuccess) {
            m_isOpened.exchange(true);
            // m_frame.reset();
            printf("[DaHua] Camera Start Play\n");
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
    bool wait_and_get(cv::Mat &frame, int64_t &timeStamp, const std::function<void()> &func) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        UINT low;
        UINT high;
        streamPtr->getFrame(m_frame, 300);
        if (m_frame.valid()) {
            /* 获得图像时需要调用的自定义函数 */
            func();

            /* cvtColor图像转换, sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_BAYGB8 */
            int64_t t = cv::getTickCount();

            cv::Mat by8Mat(cv::Size(m_frame.getImageWidth(), m_frame.getImageHeight()), CV_8UC1, (uchar *)m_frame.getImage());
            cv::cvtColor(by8Mat, frame, cv::COLOR_BayerBG2BGR_EA);
            // RG -> B
            // GR -> G
            // NO RB , BR
            // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

            //                /* sdk函数图像转换 */
            //                CameraImageProcess(m_hCamera, m_pbyBuffer, m_pRgbBuffer,
            //                &sFrameInfo);
            //                /* 刷新图像 */
            //                frame.data = m_pRgbBuffer;

            printf("[DaHua] convert rgb cost: %.2f ms\n",
                1000 * (cv::getTickCount() - t) / cv::getTickFrequency());

            if (m_isEnableRecord) {
                std::unique_lock<std::mutex> lock(m_recordLock, std::try_to_lock);
                if (lock.owns_lock()) {
                    frame.copyTo(m_recordFrame);
                    m_record = true;
                    lock.unlock();
                    m_cond.notify_one();
                }
            }

            /* 获得时间戳 */
            timeStamp = m_frame.getImageTimeStamp() / 1000;
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
