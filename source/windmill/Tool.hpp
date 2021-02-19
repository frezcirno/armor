//2021 1 26
#ifndef TOOL_HPP
#define TOOL_HPP

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#define Rad2Deg 57.295779513082 //弧度转角度

namespace wm {

    //旋转的顺序 先yaw还是先pitch
    typedef enum {
        YAW_PITCH = 0, 
        PITCH_YAW = 1
    } ROTATION_ORDER;

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数通过pitch yaw和旋转顺序计算旋转矩阵
    // 输入：pitch yaw和旋转顺序
    // 输出：旋转矩阵
    // ---------------------------------------------------------------------------------------------------------------------
    void calRvTv(const float yaw, const double pitch, cv::Mat &Rv, ROTATION_ORDER order) 
    {
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

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数利用solvePnP计算旋转矩阵和平移向量
    // 输入：
    // 输出：旋转矩阵和平移向量
    // ---------------------------------------------------------------------------------------------------------------------
    void calRvTv(const std::vector<cv::Point2f> &inputPts, const float width, const float height, 
                 const cv::Mat &camMatrix, const cv::Mat &distCoeffs, cv::Mat &Rv, cv::Mat &Tv) 
    {
        assert(inputPts.size() == 4);
        cv::Mat Ev;
        cv::Mat outputMat;
        std::vector<cv::Point3f> cornerPts;
        cornerPts.emplace_back(cv::Point3f(0, height, 0));
        cornerPts.emplace_back(cv::Point3f(0, 0, 0));
        cornerPts.emplace_back(cv::Point3f(width, 0, 0));
        cornerPts.emplace_back(cv::Point3f(width, height, 0));
        cv::solvePnP(cornerPts, inputPts, camMatrix, distCoeffs, Ev, Tv);
        //旋转向量转旋转矩阵
        cv::Rodrigues(Ev, Rv);
    };

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数通过旋转矩阵和平移向量计算在另一坐标系的坐标
    // 输入：旋转矩阵和平移向量
    // 输出：在另一坐标系的坐标
    // ---------------------------------------------------------------------------------------------------------------------
    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt, const cv::Mat &Rv, const cv::Mat &Tv) 
    {
        cv::Mat inputMat = (cv::Mat_<double>(3, 1) << inputPt.x, inputPt.y, inputPt.z);
        cv::Mat outputMat = Rv * inputMat + Tv;

        outputPt.x = (float) outputMat.at<double>(0, 0);
        outputPt.y = (float) outputMat.at<double>(0, 1);
        outputPt.z = (float) outputMat.at<double>(0, 2);
    }

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数通过旋转矩阵计算在另一坐标系的坐标
    // 输入：旋转矩阵
    // 输出：在另一坐标系的坐标
    // ---------------------------------------------------------------------------------------------------------------------
    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt, const cv::Mat &Rv) 
    {
        cv::Mat inputMat = (cv::Mat_<double>(3, 1) << inputPt.x, inputPt.y, inputPt.z);
        cv::Mat outputMat = Rv * inputMat;

        outputPt.x = (float) outputMat.at<double>(0, 0);
        outputPt.y = (float) outputMat.at<double>(0, 1);
        outputPt.z = (float) outputMat.at<double>(0, 2);
    }

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数计算装甲板中心在云台坐标系的坐标
    // 输入：
    // 输出：装甲板中心在云台坐标系的坐标
    // ---------------------------------------------------------------------------------------------------------------------
    void coodiTrans(const std::vector<cv::Point2f> &inputPts, cv::Point3f &outputPt,
               const float width, const float height,
               const cv::Mat &camMatrix, const cv::Mat &distCoeffs) 
    {
        cv::Mat Rv, Tv;
        auto inputPt = cv::Point3f(width / 2, height / 2, 0.0);
        // 此函数利用solvePnP计算旋转矩阵和平移向量
        calRvTv(inputPts, width, height, camMatrix, distCoeffs, Rv, Tv);
        // 此函数通过旋转矩阵和平移向量计算在另一坐标系的坐标
        coodiTrans(inputPt, outputPt, Rv, Tv);
    }

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数从当前坐标系转化为旋转yaw pitch的坐标系，yaw pitch为角度值
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    void coodiTrans(const cv::Point3f &inputPt, cv::Point3f &outputPt,
                    const float yaw, const float pitch,
                    ROTATION_ORDER order) 
    {
        cv::Mat Rv;
        calRvTv(yaw, pitch, Rv, order);
        coodiTrans(inputPt, outputPt, Rv);
    }

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数从三维装甲板坐标系转化为二维图片坐标系
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    void coodiTrans(const cv::Point3f &inputPt, cv::Point2f &outputPt,
                    const cv::Mat &Rv, const cv::Mat &Tv,
                    const cv::Mat &camMatrix, const cv::Mat &distCoeffs) 
    {
        std::vector<cv::Point3d> vecPt3dArmor;
        vecPt3dArmor.push_back(inputPt);
        cv::Mat result = cv::Mat::zeros(2, 1, CV_64F);
        projectPoints(vecPt3dArmor, Rv, Tv, camMatrix, distCoeffs, result);
        outputPt.x = result.at<double>(0, 0);
        outputPt.y = result.at<double>(0, 1);
    }

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数利用三维坐标点计算pitch和yaw值
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    void calPitchYaw(const cv::Point3f &inputPt, float &pitch, float &yaw) 
    {
        pitch = (float) acos(sqrt(pow(inputPt.z, 2) + pow(inputPt.x, 2)) / cv::norm(inputPt));  // pitch
        pitch = inputPt.y < 0 ? pitch : -pitch;

        yaw = (float) acos(sqrt(pow(inputPt.z, 2) + pow(inputPt.y, 2)) / cv::norm(inputPt));  // yaw
        yaw = inputPt.x > 0 ? yaw : -yaw;
        yaw *= (float) 57.295779513082;    // yaw
        pitch *= (float) 57.295779513082;  // pitch
    }
}
#endif