// 2019/3/16 by yaw
#ifndef TOOL_HPP
#define TOOL_HPP

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace wm {
    typedef enum {
        YAW_PITCH = 0, PITCH_YAW = 1
    } ROTATION_ORDER;

/*计算旋转平移矩阵*/
    void calRvTv(const float yaw, const double pitch, cv::Mat &Rv, ROTATION_ORDER order) {
        float tYaw = yaw * M_PI / (180.0);
        float tPitch = pitch * M_PI / (180.0);

        cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                0, cos(tPitch), sin(tPitch),
                0, -sin(tPitch), cos(tPitch));

        cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(tYaw), 0, -sin(tYaw),
                0, 1, 0,
                sin(tYaw), 0, cos(tYaw));

        if (order == YAW_PITCH)
            Rv = Rx * Ry;

        else if (order == PITCH_YAW)
            Rv = Ry * Rx;

    }

    void
    calRvTv(const std::vector<cv::Point2f> &inputPts, const float width, const float height, const cv::Mat &camMatrix,
            const cv::Mat &distCoeffs, cv::Mat &Rv, cv::Mat &Tv) {
        assert(inputPts.size() == 4);
        cv::Mat Ev;
        cv::Mat outputMat;
        std::vector<cv::Point3f> cornerPts;
        cornerPts.emplace_back(cv::Point3f(0, height, 0));
        cornerPts.emplace_back(cv::Point3f(0, 0, 0));
        cornerPts.emplace_back(cv::Point3f(width, 0, 0));
        cornerPts.emplace_back(cv::Point3f(width, height, 0));
        cv::solvePnP(cornerPts, inputPts, camMatrix, distCoeffs, Ev, Tv);
        cv::Rodrigues(Ev, Rv);
    };

/*经过旋转和平移*/
    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt, const cv::Mat &Rv, const cv::Mat &Tv) {

        cv::Mat inputMat = (cv::Mat_<double>(3, 1)
                << inputPt.x, inputPt.y, inputPt.z);

        cv::Mat outputMat = Rv * inputMat + Tv;

        outputPt.x = (float) outputMat.at<double>(0, 0);
        outputPt.y = (float) outputMat.at<double>(0, 1);
        outputPt.z = (float) outputMat.at<double>(0, 2);
    }

    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt, const cv::Mat &Rv) {
        cv::Mat inputMat = (cv::Mat_<double>(3, 1)
                << inputPt.x, inputPt.y, inputPt.z);
        cv::Mat outputMat = Rv * inputMat;
        outputPt.x = (float) outputMat.at<double>(0, 0);
        outputPt.y = (float) outputMat.at<double>(0, 1);
        outputPt.z = (float) outputMat.at<double>(0, 2);
    }

/*像素到装甲片坐标系， 原点为左上角, width 为x轴*/
    void coodiTrans(const std::vector<cv::Point2d> &inputPts, cv::Point3f &outputPt, float width, float height) {
        assert(inputPts.size() == 4);
        outputPt = cv::Point3f(width / 2, height / 2, 0.0);
    }

/*像素到云台*/
    void
    coodiTrans(const std::vector<cv::Point2f> &inputPts, cv::Point3f &outputPt, const float width, const float height,
               const cv::Mat &camMatrix, const cv::Mat &distCoeffs) {
        cv::Mat Rv, Tv;
        auto inputPt = cv::Point3f(width / 2, height / 2, 0.0);
        calRvTv(inputPts, width, height, camMatrix, distCoeffs, Rv, Tv);
        coodiTrans(inputPt, outputPt, Rv, Tv);
    }

/*从当前坐标系转化为旋转yaw，pitch的坐标系，yaw，pitch为角度值*/
    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt, const float yaw, const float pitch,
                    ROTATION_ORDER order) {
        cv::Mat Rv;
        calRvTv(yaw, pitch, Rv, order);
        coodiTrans(inputPt, outputPt, Rv);
    }

/*装甲片坐标系的三维点到像素*/
    void coodiTrans(const cv::Point3f &inputPt, cv::Point2f &outputPt, const cv::Mat &Rv, const cv::Mat &Tv,
                    const cv::Mat &camMatrix,
                    const cv::Mat &distCoeffs) {
        std::vector<cv::Point3d> vecPt3dArmor;
        vecPt3dArmor.push_back(inputPt);
        cv::Mat result = cv::Mat::zeros(2, 1, CV_64F);
        projectPoints(vecPt3dArmor, Rv, Tv, camMatrix, distCoeffs, result);
        outputPt.x = result.at<double>(0, 0);
        outputPt.y = result.at<double>(0, 1);
    }

    void calPitchYaw(const cv::Point3f &inputPt, float &pitch, float &yaw) {
        pitch = (float) acos(sqrt(pow(inputPt.z, 2) + pow(inputPt.x, 2)) / cv::norm(inputPt));  // pitch
        pitch = inputPt.y < 0 ? pitch : -pitch;

        yaw = (float) acos(sqrt(pow(inputPt.z, 2) + pow(inputPt.y, 2)) / cv::norm(inputPt));  // yaw
        yaw = inputPt.x > 0 ? yaw : -yaw;
        yaw *= (float) 57.295779513082;    // yaw
        pitch *= (float) 57.295779513082;  // pitch

    }
}
#endif
