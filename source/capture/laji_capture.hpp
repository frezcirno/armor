#pragma once

#include "capture/base_capture.hpp"

/**
 * 辣鸡摄像头
 */
class LajiVision : public Capture {
  private:
    cv::VideoCapture m_cap;
    int64_t m_startTickCount;
    int64_t m_currentTimeStamp;

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
            if (m_cap.open(filename, cv::CAP_FFMPEG)) {
                PRINT_INFO("[Capture] Open a video\n");
            } else {
                PRINT_ERROR("[Capture] Open a video failed!\n");
                exit(-1);
            }
        } else {
            PRINT_INFO("[Capture] Open the laji camera\n");
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
    bool wait_and_get(cv::Mat &frame, int64_t &timeStamp, const std::function<void()> &func) override {
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        m_cap >> frame;
        resize(frame, frame, cv::Size(1920, 1280));

        // frame = frame(cv::Rect(stFrameInfo.offset, stFrameInfo.size));
        func();
        timeStamp = int64_t(1000.0 * 1000.0 * (cv::getTickCount() - m_startTickCount) / cv::getTickFrequency());
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
