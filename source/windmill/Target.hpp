//2021 1 26
#ifndef TARGET_HPP
#define TARGET_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace wm
{
    //----------------------------------------------------------------------------------------------------------------------
    // 目标类
    //    为了保持接口不变，保留了之前的数据成员，但是很多没有用到
    // ---------------------------------------------------------------------------------------------------------------------
    class Target
    {
    public:
        std::vector<cv::Point2f> vertexs; // 旋转矩形的四个顶点
        double timeStamp = 0;             // 相机时间戳
        cv::Point3f lPt;                  // 云台坐标系坐标
        cv::Point3f wPt;                  // 世界坐标系坐标
        cv::Point3f tPt;                  // 装甲板坐标系坐标

        float lYaw = 0.0;    // 
        float lPitch = 0.0;  //
        float wYaw = 0.0;    // 参考系下实际真实yaw
        float wPitch = 0.0;  // 参考系下实际真实yaw

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
        ~Target() {}

        void clear() { vertexs.clear(); }
        Target &operator = (const Target &c) {
            for (int i = 0; i < 4; i++)
                vertexs.push_back(c.vertexs[i]);
            timeStamp = c.timeStamp; // 相机时间戳

            lPt = c.lPt;
            wPt = c.wPt;             // 世界坐标系坐标
            tPt = c.tPt;             // 装甲板坐标系坐标

            lYaw = c.lYaw;
            lPitch = c.lPitch;

            wYaw = c.wYaw;           // 参考系下实际真实yaw
            wPitch = c.wPitch;       // 参考系下实际真实yaw

            type = c.type;           // -1 未识别 0 = 已被打中 1 = 该打

            RvTtoL = c.RvTtoL.clone();
            TvTtoL = c.TvTtoL.clone();

            wVx = c.wVx;
            wVy = c.wVy;
            wVz = c.wVz;

            lVx = c.lVx;
            lVy = c.lVy;
            lVz = c.lVz;

            tVx = c.tVx;
            tVy = c.tVy;
            tVz = c.tVz;

            width = c.width;
            height = c.height;

            return *this;
        }
    };
}
#endif