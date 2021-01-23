// by yaw 2019/03/14

#ifndef TARGET_HPP
#define TARGET_HPP

#include <vector>
#include <opencv2/core.hpp>


namespace wm {
    struct Feature {
    };

    class Target {
    public:
        std::vector<cv::Point2f> pixelPts2f;
        double timeStamp = 0; // 照相的时间戳
        cv::Point3f lPt;  // 云台坐标系下坐标
        cv::Point3f wPt;  // 世界系下坐标
        cv::Point3f tPt;  // 装甲片系下坐标
        float lYaw = 0.0;
        float lPitch = 0.0;
        float wYaw = 0.0;  // 参考系下实际真实yaw
        float wPitch = 0.0;  // 参考系下实际真实yaw
        // Feature feature;
        int type = -1;  // -1 未识别 0 = 已被打中 1 = 该打

        cv::Mat RvTtoL;
        cv::Mat TvTtoL; //装甲片坐标系到云台坐标系

        float wVx = 0;
        float wVy = 0;
        float wVz = 0;

        float lVx = 0;
        float lVy = 0;
        float lVz = 0;

        float tVx = 0;
        float tVy = 0;
        float tVz = 0;

        float width;
        float height;

        Target() {}

        Target &operator=(const Target &c) {
            for (int i = 0; i < 4; i++) {
                pixelPts2f.push_back(c.pixelPts2f[i]);
            }
            timeStamp = c.timeStamp; // 照相的时间戳
            lPt = c.lPt;  // 相机坐标系下坐标
            wPt = c.wPt;  // 世界系下坐标
            tPt = c.tPt;  // 装甲片系下坐标
            lYaw = c.lYaw;
            lPitch = c.lPitch;
            wYaw = c.wYaw;  // 参考系下实际真实yaw
            wPitch = c.wPitch;  // 参考系下实际真实yaw
            // memcpy(&(this->feature), &(c.feature),sizeof(c.feature));
            type = c.type;  // -1 未识别 0 = 已被打中 1 = 该打

            wVx = c.wVx;
            wVy = c.wVy;
            wVz = c.wVz;

            lVx = c.lVx;
            lVy = c.lVy;
            lVz = c.lVz;

            tVx = c.tVx;
            tVy = c.tVy;
            tVz = c.tVz;

            RvTtoL = c.RvTtoL.clone();
            TvTtoL = c.TvTtoL.clone();

            width = c.width;
            height = c.height;

            return *this;
        }

        void clear() {
            pixelPts2f.clear();
        }

        ~Target() {}


    };
}
#endif  // TARGET_HPP
