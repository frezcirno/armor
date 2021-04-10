#pragma once

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>

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
    std::atomic<int64_t> m_currentInterval;  // us

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
    virtual bool init() { return true; }

    /**
     * 摄像头开始传输图像
     * @return
     */
    virtual bool play() { return true; }

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
    virtual bool isOpened() { return true; }

    /**
     * 阻塞获取摄像头, 多线程可用
     * @param frame
     * @param timeStamp 时间戳 us
     * @param func
     * @return 是否有效
     */
    virtual bool wait_and_get(cv::Mat &frame, int64_t &timeStamp, const std::function<void()> &func) { return true; }

    /**
     * 获得当前 wait_and_get 函数获得的图像时间间隔
     * @return 采集时间间隔us
     */
    int64_t getCurrentInterval() { return m_currentInterval.load(); }

    /**
     * 释放摄像头, 析构时自动调用
     */
    virtual void release() {}
};

