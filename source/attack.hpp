#pragma once

#include <atomic>
#include <deque>
#include <dirent.h>
#include <future>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

#include "ArmorFinder.hpp"
#include "TfClassifier.hpp"
#include "base.hpp"
#include "capture/capture.hpp"
#include "communicator.hpp"
#include "imageshow.hpp"
#include "kalman.hpp"
#include "pid.hpp"
#include "sort/sort.h"
#include "target.hpp"

const unsigned int max_age = 10;
const unsigned int min_hits = 1;
const double iou_threshold = 0.1;
const int height_threshold = 0;    //前后两帧装甲板中心高度差阈值
const int weight_threshold = 0;    //前后两帧装甲板中心宽度差阈值
const int drop_threshold = 0;      //掉帧系数阈值
const int continue_threshold = 0;  //持续识别系数阈值
const int anti_top_threshold = 0;  //陀螺系数阈值

/**
 * 自瞄基类, 多线程共享变量用
 */
class AttackBase {
  protected:
    static std::mutex s_mutex;                      // 互斥锁
    static std::atomic<int64_t> s_latestTimeStamp;  // 已经发送的帧编号
    static std::deque<Target> s_historyTargets;     // 打击历史, 最新的在头部, [0, 1, 2, 3, ....]
    static ArmorFinder s_armorFinder;
    static TfClassifier s_tfClassifier;

    Target target_box, last_box;  // 目标装甲板
    int drop_frame = 0;           //掉帧系数
    int anti_top = 0;             //陀螺系数
    int continue_identify = 0;    //持续识别系数

    static Kalman kalman;                              // 卡尔曼滤波
    static std::unique_ptr<sort::SORT> s_sortTracker;  // DeepSORT 跟踪
    static size_t s_trackId;                           // DeepSORT 跟踪对象Id
};
/* 类静态成员初始化 */
std::mutex AttackBase::s_mutex;
std::atomic<int64_t> AttackBase::s_latestTimeStamp(0);
std::deque<Target> AttackBase::s_historyTargets;
ArmorFinder AttackBase::s_armorFinder;
TfClassifier AttackBase::s_tfClassifier;
Kalman AttackBase::kalman;
decltype(AttackBase::s_sortTracker) AttackBase::s_sortTracker(std::make_unique<sort::SORT>(iou_threshold, max_age, min_hits));
size_t AttackBase::s_trackId;

/**
 * 自瞄主类
 */
class Attack : AttackBase {
  private:
    Communicator &m_communicator;
    ImageShowClient &m_is;
    cv::Mat m_bgr;      // ROI图
    cv::Mat m_bgr_raw;  // 原图
    // 目标
    std::vector<Target> m_preTargets;  // 预检测目标
    std::vector<Target> m_targets;     // 本次有效目标集合
    // 开小图
    cv::Point2i m_startPt;   // ROI左上角点坐标
    bool m_isEnablePredict;  // 是否开预测

    int64_t m_currentTimeStamp = 0;  // 当前时间戳
    PID &m_pid;                      // PID

  public:
    explicit Attack(Communicator &communicator, PID &pid, ImageShowClient &isClient) : m_communicator(communicator),
                                                                                       m_is(isClient),
                                                                                       m_isEnablePredict(true),
                                                                                       m_pid(pid) {
        s_armorFinder.useDialte(stConfig.get<bool>("auto.is-dilate"));
    }
    void setMode(bool colorMode) { s_armorFinder.colorMode(colorMode); }

  private:
    /**
     * Get the distance between a sort::Track and a Target
     */
    static float distance(const sort::Track &track, const Target &target) {
        auto dist1 = track.bbox.tl() - target.pixelPts2f.tl;
        auto dist2 = track.bbox.br() - target.pixelPts2f.br;
        return abs(dist1.x) + abs(dist1.y) + abs(dist2.x) + abs(dist2.y);
    }

    /**
     * 击打策略函数
     * @change s_historyTargets 在数组开头添加本次打击的目标
     * @return emSendStatusA 
     * SEND_STATUS_AUTO_AIM         击打，且找到上一次击打的目标
     * SEND_STATUS_AUTO_AIM_FORMER  击打，但是击打的是新目标
     * SEND_STATUS_AUTO_NOT_FOUND   不击打
     */
    emSendStatusA m_match() {
        /* 更新下相对帧编号 */
        for (auto iter = s_historyTargets.begin(); iter != s_historyTargets.end(); iter++) {
            iter->rTick++;
            /* 超过30帧就删除 */
            if (iter->rTick > 5) {
                s_historyTargets.erase(iter, s_historyTargets.end());
                if (s_historyTargets.empty()) s_trackId = -1;
                break;
            }
        }

        /* Tracker 更新 */
        std::vector<sort::BBox> bboxs;
        for (auto &&tar : m_targets) {
            bboxs.emplace_back(tar.pixelPts2f.toRect());
        }
        std::vector<sort::Track> tracks = s_sortTracker->update(bboxs);

        m_is.addTracks(tracks);
        m_is.addText(cv::format("Track Id: %ld", s_trackId));

        if (m_targets.empty()) {
            // 这一帧没找到目标
            // 如果30帧内有历史目标则打历史目标，否则返回未找到
            return s_historyTargets.empty() ?
                       SEND_STATUS_AUTO_NOT_FOUND :
                       SEND_STATUS_AUTO_AIM_FORMER;

        } else if (s_historyTargets.empty()) {
            // 之前没选择过打击目标
            // 找到含离云台最近的目标, 返回瞄准且找到
            Target nearestTarget = *std::min_element(
                m_targets.begin(), m_targets.end(), [](const Target &a_, const Target &b_) {
                    return cv::norm(a_.ptsInGimbal) < cv::norm(b_.ptsInGimbal);
                });
            s_historyTargets.emplace_front(nearestTarget);
            // 匹配最近的trackId并记录下来
            auto minTrackElem = std::min_element(tracks.begin(), tracks.end(), [&](const sort::Track &a, const sort::Track &b) {
                return distance(a, nearestTarget) < distance(b, nearestTarget);
            });
            s_trackId = (minTrackElem != tracks.end() ? minTrackElem->id : s_trackId);
            return SEND_STATUS_AUTO_AIM;

        } else if (s_trackId != -1) {
            // 之前选择过打击目标, 而且tracker没有丢失
            // 寻找最接近上一次目标的目标
            // 如果找到: 返回瞄准且找到
            // 否则: 返回瞄准且找到
            auto trackElem =
                std::find_if(tracks.begin(), tracks.end(), [&](const sort::Track &track) {
                    return track.id == s_trackId;
                });

            if (trackElem != tracks.end()) {
                // 找到track对象
                Target closestTarget = *std::min_element(m_targets.begin(), m_targets.end(), [&](const Target &a, const Target &b) {
                    return distance(*trackElem, a) < distance(*trackElem, b);
                });
                s_historyTargets.emplace_front(closestTarget);
                return SEND_STATUS_AUTO_AIM;  //瞄准上一帧
            } else {
                return SEND_STATUS_AUTO_AIM_FORMER;  //瞄准上一帧
            }
        } else {
            // 之前选择过打击目标, 但是tracker已经丢失
            // 因为s_historyTargets还有值，暂时使用旧值
            return SEND_STATUS_AUTO_AIM_FORMER;  //瞄准上一帧
        }
        return SEND_STATUS_AUTO_NOT_FOUND;
    }

  public:
    /**
     * @param enable = true: 开启
     * 设置是否开启预测
     */
    void enablePredict(bool enable = true) {
        m_communicator.enableReceiveGlobalAngle(enable);
        m_isEnablePredict = enable;
    }

    /**
     * @param tar 上一个检测到的装甲
     * @param rect 截的图
     * @param size 采集图像参数
     * @param extendFlag 是否扩展
     * 图像扩展ROI
     */
    static void getBoundingRect(Target &tar, cv::Rect &rect, cv::Size &size, bool extendFlag = false) {
        rect = cv::boundingRect(s_historyTargets[0].pixelPts2f_Ex.toArray());

        if (extendFlag) {
            rect.x -= int(rect.width * 4);
            rect.y -= rect.height * 3;
            rect.width *= 9;
            rect.height *= 7;

            rect.width = rect.width >= size.width ? size.width - 1 : rect.width;
            rect.height = rect.height >= size.height ? size.height - 1 : rect.height;

            rect.width = rect.width < 80 ? 80 : rect.width;
            rect.height = rect.height < 50 ? 50 : rect.height;

            rect.x = rect.x < 1 ? 1 : rect.x;
            rect.y = rect.y < 1 ? 1 : rect.y;

            rect.width = rect.x + rect.width >= size.width ? size.width - 1 - rect.x : rect.width;
            rect.height = rect.y + rect.height >= size.height ? size.height - 1 - rect.y : rect.height;
        }
    }

    /**
     * 判断对手是否开启小陀螺
     */
    bool is_antitop() {
        if (!s_historyTargets.empty()) {
            cv::Point2f center = target_box.pixelCenterPt2f;
            cv::Point2f center_ = last_box.pixelCenterPt2f;
            if (abs(center.y - center_.y) < height_threshold) {
                if (abs(center.x - center_.x) < weight_threshold) {
                    anti_top++;
                } else {
                    continue_identify++;
                    if (continue_identify > continue_threshold) {
                        drop_frame = 0;
                    }
                }
            } else {
                anti_top = 0;
            }
        } else {
            drop_frame++;
            if (drop_frame > drop_threshold) {
                anti_top = 0;
                continue_identify = 0;
            }
        }
        if (anti_top > anti_top_threshold)
            return true;
        else
            return false;
    }

    /**
     * 主运行函数
     * @param src 彩图
     * @param timeStamp 调用时的时间戳
     * @param gYaw 从电控获得yaw
     * @param gPitch 从电控获得pitch
     * @return true
     */
    bool run(cv::Mat &src, int64_t timeStamp, float gYaw, float gPitch) {
        using namespace std::chrono_literals;

        m_is.addText(cv::format("gPitch %4f", gPitch));
        m_is.addText(cv::format("gYaw %4f", gYaw));

        /* 1.初始化参数，判断是否启用ROI */
        m_bgr_raw = src;
        m_bgr = src;
        m_currentTimeStamp = timeStamp;
        m_targets.clear();
        m_preTargets.clear();
        m_startPt = cv::Point();
        float delay_time = 0.0;  //延迟射击的时间

        /* 如果有历史打击对象 */
        if (s_historyTargets.size() >= 2 && s_historyTargets[0].rTick <= 10) {
            cv::Rect latestShootRect;
            getBoundingRect(s_historyTargets[0], latestShootRect, stFrameInfo.size, true);
            m_is.addRect("Bounding Rect", latestShootRect);
            m_bgr = m_bgr(latestShootRect);
            m_startPt = latestShootRect.tl();
        }
        m_is.addText(cv::format("Start Point: %2d %2d", m_startPt.x, m_startPt.y));

        /* 2.预检测 */
        s_armorFinder.detect(m_bgr, m_preTargets, m_is, m_startPt);

        /* 3.通过分类器 */
        m_is.clock("m_classify");
        s_tfClassifier.m_classify_single_tensor(m_bgr_raw, m_preTargets, m_targets, m_is);
        m_is.clock("m_classify");

        /* 如果已经有更新的一帧发出去了, 则取消本帧的发送 */
        if (timeStamp < s_latestTimeStamp.load())
            return false;

        /* 取得发送锁🔒 */
        std::unique_lock<std::mutex> preLock(s_mutex, std::try_to_lock);
        while (!preLock.owns_lock() && timeStamp > s_latestTimeStamp.load()) {
            std::this_thread::sleep_for(5us);
            preLock.try_lock();
        }

        /* 目标匹配 + 预测 + 修正弹道 + 计算欧拉角 + 射击策略 */
        if (preLock.owns_lock() && timeStamp > s_latestTimeStamp.load()) {
            s_latestTimeStamp.exchange(timeStamp);
            float rYaw = 0.0, rPitch = 0.0;  // 相对Yaw和Pitch

            /* 获得云台全局欧拉角 */
            m_communicator.getGlobalAngle(&gYaw, &gPitch);

            /* 计算世界坐标参数，转换到世界坐标系 */
            for (auto &tar : m_targets) {
                tar.calcWorldParams();                // 计算云台坐标系坐标
                tar.convert2WorldPts(-gYaw, gPitch);  // 计算世界坐标系坐标
            }
            /* 4.目标匹配 */
            emSendStatusA statusA = m_match();

            if (!s_historyTargets.empty()) {
                m_is.addFinalTargets("final", s_historyTargets[0]);
                /* 5.修正弹道并计算欧拉角 */
                float bulletSpeed;
                float finalPitch = 0;
                m_communicator.getBulletSpeed(&bulletSpeed);
                double vdistance = 0.001 * cv::sqrt(s_historyTargets[0].ptsInWorld.x * s_historyTargets[0].ptsInWorld.x + s_historyTargets[0].ptsInWorld.z * s_historyTargets[0].ptsInWorld.z);  // 水平方向距离，单位m
                double hdistance = 0.001 * -s_historyTargets[0].ptsInWorld.y;
                s_historyTargets[0].correctTrajectory_and_calcEuler(bulletSpeed, gPitch, finalPitch);
                m_is.addText(cv::format("vdistance %4f", vdistance));
                m_is.addText(cv::format("hdistance %4f", hdistance));
                m_is.addText(cv::format("finalPitch %4f", finalPitch));
                rYaw = s_historyTargets[0].rYaw;
                rPitch = s_historyTargets[0].rPitch;
                target_box = s_historyTargets[0];

                /* 6.预测部分 */
                if (m_isEnablePredict) {
                    m_is.addText(cv::format("b4pdct rPitch %4f", rPitch));
                    m_is.addText(cv::format("b4pdct rYaw %4f", rYaw));
                    // 初始化卡尔曼滤波
                    if (statusA == SEND_STATUS_AUTO_AIM) {
                        // 若找到上一次目标, 或者首次选择目标
                        m_communicator.getGlobalAngle(&gYaw, &gPitch);
                        if (s_historyTargets.size() == 1)
                            // 首次选择目标
                            kalman.clear_and_init(rPitch, rYaw, timeStamp);
                        else {
                            // 若找到上一次目标
                            kalman.correct(rPitch, rYaw, timeStamp);
                        }
                    }
                    // 进行预测和坐标修正
                    if (s_historyTargets.size() > 1) {
                        kalman.predict(0.1, s_historyTargets[0].predictPitch, s_historyTargets[0].predictYaw);
                        /* 转换为云台坐标点 */
                        m_is.addText(cv::format("predictPitch %4f", s_historyTargets[0].predictPitch));
                        m_is.addText(cv::format("predictYaw %4f", s_historyTargets[0].predictYaw));
                        rYaw = s_historyTargets[0].predictYaw;
                        rPitch = s_historyTargets[0].predictPitch;
                    }
                }

                if (s_historyTargets.size() >= 2) {
                    target_box = s_historyTargets[0];
                    last_box = s_historyTargets[0];
                }

                // // 7.射击策略
                // // 目标被识别3帧以上才打
                //
                // if (s_historyTargets.size() >= 3)
                //     statusA = SEND_STATUS_AUTO_SHOOT;  //射击

                m_is.addText(cv::format("ptsInGimbal: %2.2f %2.2f %2.2f",
                    s_historyTargets[0].ptsInGimbal.x / 1000.0,
                    s_historyTargets[0].ptsInGimbal.y / 1000.0,
                    s_historyTargets[0].ptsInGimbal.z / 1000.0));
                m_is.addText(cv::format("ptsInWorld: %2.2f %2.2f %2.2f",
                    s_historyTargets[0].ptsInWorld.x / 1000.0,
                    s_historyTargets[0].ptsInWorld.y / 1000.0,
                    s_historyTargets[0].ptsInWorld.z / 1000.0));
            }

            /* 8.反小陀螺模式*/
            bool is_anti = is_antitop();
            m_is.addText(cv::format("is_antitop   %.3d", is_anti));
            if (is_anti) {
                float width = 0.0;
                if (target_box.type == TARGET_SMALL)
                    width = 135;
                else
                    width = 230;
                if (cv::norm(last_box.pixelCenterPt2f - target_box.pixelCenterPt2f) > width * 1.5) {
                    cv::Point3d center = last_box.ptsInGimbal;
                    cv::Point3d center_ = target_box.ptsInGimbal;
                    cv::Point3d target = cv::Point3d((center.x + center_.x) / 2, (center.y + center_.y) / 2, (center.z + center_.z) / 2);
                    float yaw = cv::fastAtan2(target.x, cv::sqrt(target.y * target.y + target.z * target.z));
                    yaw = yaw > 180 ? yaw - 360 : yaw;
                    rYaw = yaw;
                    rPitch = cv::fastAtan2(target.y, cv::sqrt(target.x * target.x + target.z * target.z));
                } else {
                    rYaw = 0;
                    rPitch = 0;
                }
            }

            m_is.addText(cv::format("rPitch %.3f", rPitch));
            m_is.addText(cv::format("rYaw   %.3f", rYaw));
            m_is.addText(cv::format("statusA   %.3x", statusA));

            /* 9.发给电控 */
            m_communicator.send(rYaw, -rPitch, statusA, SEND_STATUS_WM_PLACEHOLDER);
            PRINT_INFO("[attack] send = %ld", timeStamp);
        }
        if (preLock.owns_lock())
            preLock.unlock();
        return true;
    }
};