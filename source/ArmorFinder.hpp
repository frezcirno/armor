#pragma once

#include <opencv2/opencv.hpp>

#include "imageshow.hpp"
#include "target.hpp"

class ArmorFinder {
    bool _colorMode = true;   // 模式:红t蓝f
    bool _useDialte = false;  // 是否膨胀

    static bool shareEdge(const Target &t1, const Target &t2) {
        if (t1.pixelPts2f.tr == t2.pixelPts2f.tl && t1.pixelPts2f.br == t2.pixelPts2f.bl) {
            return true;
        }
        if (t2.pixelPts2f.tr == t1.pixelPts2f.tl && t2.pixelPts2f.br == t1.pixelPts2f.bl) {
            return true;
        }
        return false;
    }

  public:
    ArmorFinder() {}

    bool colorMode() const { return _colorMode; }
    void colorMode(bool val) { _colorMode = val; }

    bool useDialte() const { return _useDialte; }
    void useDialte(bool val) { _useDialte = val; }

    /**
     * 通过hsv筛选和进行预处理获得装甲板
     * @change m_preTargets 预检测得到的装甲板列表, 可能有两个装甲板共享一个灯条的情况发生
     */
    void detect(const cv::Mat &bgr, std::vector<Target> &m_preTargets, ImageShowClient &m_is, const cv::Point2f &m_startPt) const {
        cv::Mat bgrChecked;

        /* 使用inRange对颜色进行筛选: bgr -> bgrChecked */
        m_is.clock("inRange");
        if (_colorMode) {
            /* 红色 */
            cv::inRange(bgr, cv::Scalar(0, 0, 140), cv::Scalar(110, 110, 255), bgrChecked);
        } else {
            /* 蓝色 */
            cv::inRange(bgr, cv::Scalar(130, 100, 0), cv::Scalar(255, 255, 65), bgrChecked);
        }
        m_is.clock("inRange");

        /* 进行膨胀操作（默认关闭）: bgrChecked -> bgrChecked */
        // m_is.addImg("bgrChecked", bgrChecked, false);
        if (_useDialte) {
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            dilate(bgrChecked, bgrChecked, element);
            m_is.addImg("dilate", bgrChecked, false);
        }

        /* 寻找边缘，并圈出contours: bgrChecked -> contours */
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(bgrChecked, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // m_is.addContours("contours", contours, m_startPt);

        /* 对contours进行筛选 */
        std::vector<Light> lights;
        for (const auto &_pts : contours) {
            // 目标不会出现在顶部
            // if (_pts.at(0).y <= 100) {
            //     continue;
            // }
            // 面积筛选
            if (_pts.size() < 5) {
                continue;
            }
            // 长度宽度筛选
            cv::RotatedRect rRect = cv::minAreaRect(_pts);
            if (rRect.size.height < 3 || rRect.size.width < 3) {
                continue;
            }
            // 面积比
            double area = cv::contourArea(_pts);
            if (area / rRect.size.area() < 0.3) {
                continue;
            }
            // 最小外接矩形长宽比2/3～3/2
            double hw = rRect.size.aspectRatio();
            if (hw > 0.666667 && hw < 1.5) {
                continue;
            }

            /* 寻找灯条的顶部中点，底部中点与倾斜角 */
            Light _light;
            cv::Point2f topPt;     //顶部中点
            cv::Point2f bottomPt;  //底部中点
            cv::Point2f pts[4];    // 四个角点
            rRect.points(pts);
            if (rRect.size.width > rRect.size.height)  //根据外接矩形的特性需调整点
            {
                bottomPt = (pts[2] + pts[3]) / 2.0;
                topPt = (pts[0] + pts[1]) / 2.0;
                _light.angle = cv::abs(rRect.angle);
            } else {
                bottomPt = (pts[1] + pts[2]) / 2;
                topPt = (pts[0] + pts[3]) / 2;
                _light.angle = cv::abs(rRect.angle - 90);
            }
            /* 判断顶部和底部中点是否设置正确，并将中心点与长度一并写入_light参数中 */
            if (topPt.y > bottomPt.y) {
                _light.topPt = bottomPt;
                _light.bottomPt = topPt;
            } else {
                _light.topPt = topPt;
                _light.bottomPt = bottomPt;
            }
            _light.centerPt = rRect.center;              //中心点
            _light.length = cv::norm(bottomPt - topPt);  //长度

            /* 判断长度和倾斜角是否合乎要求 */
            if (_light.length < 3.0 || 90.0 < _light.length || cv::abs(_light.angle - 90) > 30.0) {
                continue;
            }

            std::cout << _light.angle << std::endl;
            lights.emplace_back(_light);
        }
        m_is.addLights("lights", lights, m_startPt);

        /* 对筛选出的灯条按x大小进行排序 */
        std::sort(lights.begin(), lights.end(), [](const Light &a_, const Light &b_) -> bool {
            return a_.centerPt.x < b_.centerPt.x;
        });

        /* 对灯条进行两两组合并筛选出预检测的装甲板 */
        for (size_t i = 0; i < lights.size(); ++i) {
            for (size_t j = i + 1; j < lights.size(); ++j) {
                const auto &li = lights[i];
                const auto &lj = lights[j];

                double maxLength = cv::max(li.length, lj.length);
                double minLength = cv::min(li.length, lj.length);
                // double avgLength = (li.length + lj.length) / 2.0;
                double deltaAngle = cv::abs(li.angle - lj.angle);
                // 两灯条倾斜角
                // if (deltaAngle > 20.0) {
                //     continue;
                // }
                cv::Vec2f crossVec = lj.centerPt - li.centerPt;
                float avgAngle = (li.angle + lj.angle) / 2.0 / 180.0 * M_PI;  // in rad
                // 垂直坐标差值
                if (li.bottomPt.y < lj.topPt.y || lj.bottomPt.y < li.topPt.y) {
                    continue;
                }
                // if (abs(crossVec[1]) > minLength) {
                //     continue;
                // }
                // 水平坐标差值
                if (abs(crossVec[0]) < minLength) {
                    continue;
                }
                if (cv::norm(crossVec) / minLength > 5) {
                    continue;
                }
                // 平行四边形度
                cv::Vec2f orientation(cos(avgAngle), sin(avgAngle));
                if (abs(orientation.dot(crossVec)) >= 25) {
                    continue;
                }
                // 长边短边比
                if (maxLength > 1.5 * minLength) {
                    continue;
                }
                // if (cv::fastAtan2(cv::abs(crossVec[1]), cv::abs(crossVec[0])) > 25.0) {
                //     continue;
                // }
                Target target;
                /* 计算像素坐标 */
                target.setPixelPts(li.topPt, li.bottomPt, lj.bottomPt, lj.topPt,
                    m_startPt);

                /** 
                 * small: 135 x 55 -> 2.454
                 * large: 230 x 55 -> 4.1818
                 */
                if (cv::norm(crossVec) / minLength > 2.5)
                    target.type = TARGET_LARGE;  // 大装甲

                // bool cancel = 0;
                // for (std::vector<Target>::iterator it = m_preTargets.begin(); it != m_preTargets.end(); it++) {
                //     const Target &t = *it;
                //     if (shareEdge(t, target)) {
                //         cv::Vec2f tLeft = t.pixelPts2f.tl - t.pixelPts2f.bl;
                //         cv::Vec2f tRight = t.pixelPts2f.tr - t.pixelPts2f.br;
                //         float angleDiffT = abs(std::atan2(tLeft[0], tLeft[1]) - std::atan2(tRight[0], tRight[1]));
                //         if (angleDiffT > deltaAngle) {
                //             m_preTargets.erase(it);
                //         } else {
                //             cancel = 1;
                //         }
                //         break;
                //     }
                // }
                // if (cancel) {
                //     continue;
                // }

                /* 获得扩展区域像素坐标, 若无法扩展则放弃该目标 */
                if (!target.convert2ExternalPts2f())
                    continue;
                m_preTargets.emplace_back(std::move(target));
            }
        }
        m_is.addTargets("preTargets", m_preTargets);
    }
};
