#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>

/**
 * 卡尔曼滤波线性预测
 * 状态量: [x, y, z, Delta_x, Delta_y, Delta_z]'
 * 观测量: [x, y, z]
 * 非线程安全
 */
class Kalman {
    cv::KalmanFilter m_kf;
    cv::Mat m_measurement = cv::Mat::zeros(2, 1, CV_32F);
    int64_t m_lastTimeStamp;

  public:
    cv::Point3d velocity;

    explicit Kalman() : m_lastTimeStamp(0) {
        m_kf.init(4, 2, 0);
        m_kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        setIdentity(m_kf.measurementMatrix);                           //观测模型
        setIdentity(m_kf.processNoiseCov, cv::Scalar::all(1e-5));      //过程噪声
        setIdentity(m_kf.measurementNoiseCov, cv::Scalar::all(1e-1));  //观测噪声
        setIdentity(m_kf.errorCovPost, cv::Scalar::all(1));            //预测估计协方差矩阵
    }
    /**
     * @param pos 输入三维坐标 
     * @param timestamp 时间戳
     * func 卡尔曼滤波清零初始化
     */
    void clear_and_init(float gPitch, float gYaw, int64_t timeStamp) {
        m_kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        setIdentity(m_kf.measurementMatrix);
        setIdentity(m_kf.processNoiseCov, cv::Scalar::all(1e-5));
        setIdentity(m_kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        setIdentity(m_kf.errorCovPost, cv::Scalar::all(1));
        m_kf.statePost = (cv::Mat_<float>(4, 1) << gPitch, gYaw, 0, 0);
        m_lastTimeStamp = timeStamp;
    }

    /**
     * @param pos 三维坐标
     * @param timeStamp 微秒
     * 卡尔曼修正函数
     */
    void correct(float *gPitch, float *gYaw, int64_t timeStamp) {
        float deltaT = (timeStamp - m_lastTimeStamp) / 10000.0;
        assert(deltaT > 0);
        m_lastTimeStamp = timeStamp;
        m_kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, deltaT, 0,
            0, 1, deltaT, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
        m_measurement.at<float>(0) = (float)*gPitch;
        m_measurement.at<float>(1) = (float)*gYaw;
        //m_measurement.at<float>(2) = (float)pos.z;
        m_kf.correct(m_measurement);
    }

    /**
     * @param delay 秒
     * @param predictRelativePos 预测坐标
     */
    void predict(float delay, cv::Point3d &predictRelativePos) {
        cv::Mat prediction = m_kf.predict();
        velocity = cv::Point3d(prediction.at<float>(3), prediction.at<float>(4), prediction.at<float>(5));
        predictRelativePos.x = prediction.at<float>(0) + delay * prediction.at<float>(3);
        predictRelativePos.y = prediction.at<float>(1) + delay * prediction.at<float>(4);
        predictRelativePos.z = prediction.at<float>(2) + delay * prediction.at<float>(5);
    }

    /**
     * @param delay 秒
     * @param predictPitch 预测角度
     * @param predictYaw 预测角度
     */
    void predict(float delay, float *predictPitch, float *predictYaw) {
        cv::Mat prediction = m_kf.predict();
        *predictPitch = prediction.at<float>(0) + delay * prediction.at<float>(2);
        *predictYaw = prediction.at<float>(1) + delay * prediction.at<float>(3);
    }
};