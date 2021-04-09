//
// Created by sp on 19-6-19.
//
#ifndef ATTACK_BASE_HPP
#define ATTACK_BASE_HPP

#include <cmath>
#include <opencv2/opencv.hpp>
#include "debug.h"
#include "config.hpp"
#include "semaphore.hpp"
//#include "capture.hpp"

const double g = 9.8; 
const double x = 0,y = 0,pitch = 0;
namespace armor {

Config stConfig("../config.toml");

/**
 * 采集的图像的参数
 */
struct FrameInfo {
    cv::Size2i size = cv::Size2i(0, 0);
    cv::Point2i offset = cv::Point2i(0, 0);
} stFrameInfo;

/**
 * 弹道拟合
 * 
 */
class DDSolver
{
  private:
    double k1 = 0.1;  // 和空气阻力有关的系数，等于(k0 / m)，pitchAdvance中用

  public:
    DDSolver(double k1 = 0.1) : k1(k1) {}

    /**
     * 只考虑重力的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位m/s
     * @param x 目标离自己的水平距离，单位m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位m
     */
    double pitchNaive(double bulletSpeed, double x, double y)
    {
        double iterY = y, pitch_;
        double diff;
        int t = 0;
        do {
            pitch_ = atan2(iterY, x);
            double vx_ = bulletSpeed * cos(pitch_);
            double vy_ = bulletSpeed * sin(pitch_);
            double t_ = x / vx_;
            double y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            iterY += diff;
            // cout << pitch_ * 180 / M_PI << " " << iterY << " " << diff <<
            // endl;
        } while (abs(diff) >= 0.001 && ++t < 30);  // 误差小于1mm
        return pitch_;
    }

    /**
     * 考虑水平空气阻力（和重力）的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位m/s
     * @param x 目标离自己的水平距离，单位m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位m
     */
    double pitchAdvance(double bulletSpeed, double x, double y)
    {
        double iterY = y, pitch_;
        double diff;
        int t = 0;
        do {
            pitch_ = atan2(iterY, x);
            double vx_ = bulletSpeed * cos(pitch_);
            double vy_ = bulletSpeed * sin(pitch_);
            double t_ = (exp(x * k1) - 1) / (k1 * vx_);
            double y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            iterY += diff;
            // cout << pitch_ * 180 / M_PI << " " << iterY << " " << diff <<
            // endl;
        } while (abs(diff) >= 0.001 && ++t < 30);  // 误差小于1mm
        return pitch_;
    }

    /**
     * 在考虑水平空气阻力（和重力）的弹道模型中，根据一组已知的四个值反推k1
     * @param bulletSpeed 弹速，单位m/s
     * @param pitch 仰角，单位rad
     * @param x 目标离自己的水平距离，单位m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位m
     */
    static double get_k1(double bulletSpeed, double pitch, double x, double y)
    {
        double vx0 = bulletSpeed * cos(pitch);
        double vy0 = bulletSpeed * sin(pitch);
        double t0 = (vy0 + sqrt(vy0 * vy0 - 2 * g * y)) / g;
        double k1 = 2 * (vx0 * t0 - x) / (x * x);
        double diff;
        double alpha = 0.01;  // 类似学习率
        int t = 0;
        do {
            double t_ = (exp(x * k1) - 1) / (k1 * vx0);
            double y_ = vy0 * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            k1 -= alpha * diff;
            // cout << k1 << " " << diff << endl;
        } while (abs(diff) >= 0.001 && ++t < 1000);  // 误差小于1mm
        return k1;
    }
};

/**
 * 相机结构体
 */
struct Camera {
    cv::Mat camMat;
    cv::Mat distCoeffs;
    /**
     * @param path 相机参数文件路径
     * 结构体构造函数
     */
    explicit Camera(const cv::String &path) {
        /* 读取相机标定文件 */
        cv::FileStorage fs(path, cv::FileStorage::READ);
        cv::read(fs["camera_matrix"], camMat);
        cv::read(fs["distortion_coefficients"], distCoeffs);
        fs.release();
    }

    /**
     * @param pts 三维坐标
     * @param pYaw
     * @param pPitch
     * 三维坐标转欧拉角工具函数
     */
    static void convertPts2Euler(cv::Point3d &pts, float *pYaw, float *pPitch) {
        float _pitch = cv::fastAtan2(pts.y, cv::sqrt(pts.x * pts.x + pts.z * pts.z));
        float _yaw = cv::fastAtan2(pts.x, cv::sqrt(pts.y * pts.y + pts.z * pts.z));
        _pitch = _pitch > 180 ? _pitch - 360 : _pitch;
        *pPitch = -_pitch;
        *pYaw = _yaw > 180 ? _yaw - 360 : _yaw;
    }
    /**
    * @param newpts 新的三维坐标
    * @param pitch 新的pitch
    * @param pts 原三维坐标
    * 三维坐标修改pitch分量工具函数
    */
    static void convertEuler2Pts(cv::Point3d &newpts, float pitch, cv::Point3d &pts) {
        float _pitch = -pitch;
        _pitch = _pitch > 0 ? _pitch : 360 + _pitch;
        newpts.x = pts.x;
        newpts.z = pts.z;
        newpts.y = tan(_pitch / 180 * CV_PI) * cv::sqrt(newpts.x * newpts.x + newpts.z * newpts.z);
    }

    /**
     * @param pts 原始坐标值
     * @param newPts 修正后坐标值
     * 考虑到重力对子弹的影响，对云台所需仰角进行补偿
     */
    void correctTrajectory(cv::Point3d &pts, cv::Point3d &newPts, float bulletSpeed) {
        double k1,newpitch;
        k1 = DDSolver::get_k1(bulletSpeed,pitch,x,y);
        DDSolver dd = DDSolver(k1);
        newpitch = dd.pitchAdvance(bulletSpeed,pts.z*0.01,pts.y*0.01);
        //newpitch = dd.pitchNaive(bulletSpeed,pts.z*0.01,pts.y*0.01);

        convertEuler2Pts(newPts, newpitch, pts);
    }
} stCamera("../data/camera6mm.xml");

// TODO: 测量, 实际检测灯长度不是55mm
/**
 * 装甲板物理参数
 */
struct
{
    /**
     * x - Right
     * y - Up
     * z - Forward
     */
    // 仅灯条矩形
    std::vector<cv::Point3d> smallFig3f = {cv::Point3d(0, 0, 0), cv::Point3d(0, -55, 0), cv::Point3d(135, -55, 0), cv::Point3d(135, 0, 0)};
    std::vector<cv::Point3d> largeFig3f = {cv::Point3d(0, 0, 0), cv::Point3d(0, -55, 0), cv::Point3d(230, -55, 0), cv::Point3d(230, 0, 0)};
    // 扩展区域
    std::vector<cv::Point2f> smallFig_Ex = {cv::Point2f(0, 0), cv::Point2f(0, 126), cv::Point2f(135, 126), cv::Point2f(135, 0)};
    std::vector<cv::Point2f> largeFig_Ex = {cv::Point2f(0, 0), cv::Point2f(0, 126), cv::Point2f(230, 126), cv::Point2f(230, 0)};
    //
    cv::Mat smallShootPosition = cv::Mat(cv::Point3d(67.5, -27.5, 0.0));
} stArmorStdFigure;

/**
 * 灯条结构体
 */
struct Light {
    cv::Point2f topPt;
    cv::Point2f bottomPt;
    cv::Point2f centerPt;
    double angle = 0;
    double length = 0;
};

typedef enum {
    TARGET_SMALL,
    TARGET_LARGE
} emTargetType;

/**
 * 四边形数据结构
 */
template <typename T>
struct Quadrilateral {
    cv::Point_<T> tl, tr, bl, br;
    inline std::vector<cv::Point_<T>> toArray() const {
        return {tl, bl, br, tr};
    }
    inline cv::Rect_<T> toRect() const {
        return {tl, br};
    }
};

/**
 */
struct Target {                          // TODO: 结构体太大了，尝试优化不必要的变量
    Quadrilateral<float> pixelPts2f;     // 硬件ROI图幅下的像素坐标（即m_bgr_raw中的坐标） TODO: 改成cv::Rect<float>类型
    cv::Point2f pixelCenterPt2f;         // 像素坐标中心
    Quadrilateral<float> pixelPts2f_Ex;  // 扩展像素坐标
    cv::Point3d ptsInGimbal;             // 物体在云台坐标系下坐标(相机坐标系经过固定变换后得到)
    cv::Point3d ptsInWorld;              // 物体在世界坐标系下坐标
    cv::Point3d ptsInWorld_Predict;      // 物体在预测后的世界坐标系下坐标, 不开启预测的时候和 ptsInWorld 一样
    cv::Point3d ptsInGimbal_Predict;     // 物体在预测后的云台坐标系下坐标, 不开启预测的时候和 ptsInGimbal 一样
    cv::Point3d ptsInShoot;              // 物体在经过弹道修正后的云台坐标系下坐标
    float predictPitch;                  // 新卡尔曼滤波
    float predictYaw;                    // 新卡尔曼滤波
    float rPitch;                        // 相对Pitch值, 发给电控
    float rYaw;                          // 相对Yaw值, 发给电控
    float bulletSpeed;                   // 子弹速度
    int rTick;                           // 相对帧编号
    emTargetType type;                   // TARGET_SMALL, TARGET_TARGET

    cv::Mat rv,  // 旋转向量
        tv,      // 偏移向量
        rvMat;   // 旋转矩阵

    cv::Mat m_rotY, m_rotX;  // 旋转到绝对坐标系
    cv::Point3d vInGimbal3d;

    explicit Target() : rPitch(0), rYaw(0), rTick(0), type(TARGET_SMALL) {}

    Target(const Target &) = default;

    Target operator=(const Target &t) {
        this->pixelPts2f = t.pixelPts2f;
        this->pixelCenterPt2f = t.pixelCenterPt2f;
        this->pixelPts2f_Ex = t.pixelPts2f_Ex;
        this->ptsInGimbal = t.ptsInGimbal;
        this->ptsInWorld = t.ptsInWorld;
        this->ptsInWorld_Predict = t.ptsInWorld_Predict;
        this->ptsInGimbal_Predict = t.ptsInGimbal_Predict;
        this->ptsInShoot = t.ptsInShoot;
        this->rPitch = t.rPitch;
        this->rYaw = t.rYaw;
        this->rTick = t.rTick;
        this->type = t.type;
        this->rv = t.rv;
        this->tv = t.tv;
        this->rvMat = t.rvMat;
        this->m_rotY = t.m_rotY;
        this->m_rotX = t.m_rotX;
        this->vInGimbal3d = t.vInGimbal3d;
        this->bulletSpeed = t.bulletSpeed;
        return *this;
    }

    /**
     * 移动构造函数
     * @param t 
     */
    Target(Target &&t) : rPitch(t.rPitch), rYaw(t.rYaw), rTick(t.rTick), type(t.type),
                         pixelPts2f(std::move(t.pixelPts2f)),
                         pixelCenterPt2f(std::move(t.pixelCenterPt2f)),
                         pixelPts2f_Ex(std::move(t.pixelPts2f_Ex)),
                         ptsInGimbal(std::move(t.ptsInGimbal)),
                         ptsInWorld(std::move(t.ptsInWorld)),
                         ptsInWorld_Predict(std::move(t.ptsInWorld_Predict)),
                         ptsInGimbal_Predict(std::move(t.ptsInGimbal_Predict)),
                         ptsInShoot(std::move(t.ptsInShoot)),
                         rv(std::move(t.rv)), tv(std::move(t.tv)), rvMat(std::move(t.rvMat)),
                         m_rotY(std::move(t.m_rotY)), m_rotX(std::move(t.m_rotX)), vInGimbal3d(std::move(t.vInGimbal3d)) {}

    /**
     * @param tl 左上（从左上开始顺时针设置）
     * @param bl 左下
     * @param br 右下
     * @param tr 右上
     * @param startPt 开小图模式下的偏移量
     * 设置硬件ROI图幅下的像素坐标,计算ROI像素中心点坐标
     */
    void setPixelPts(const cv::Point2f &tl, const cv::Point2f &bl, const cv::Point2f &br, const cv::Point2f &tr, const cv::Point2f &startPt) {
        //计算硬件ROI图幅下的像素坐标
        pixelPts2f.tl = tl + startPt;
        pixelPts2f.tr = tr + startPt;
        pixelPts2f.bl = bl + startPt;
        pixelPts2f.br = br + startPt;
        //逐个遍历，将像素坐标累积求和
        pixelCenterPt2f = (pixelPts2f.tl + pixelPts2f.tr + pixelPts2f.bl + pixelPts2f.br) / 4;
    }

    /**
     * @return false = 扩展后超过硬件ROI图幅大小
     * 将灯条拓展，如果拓展后超范围，返回false,否则返回true
     */
    bool convert2ExternalPts2f() {
        //清除掉扩展像素坐标向量的元素，不释放内存
        cv::Point2f halfDeltaA = (pixelPts2f.tl - pixelPts2f.bl) / 55 * 35;
        pixelPts2f_Ex.tl = pixelPts2f.tl + halfDeltaA;
        pixelPts2f_Ex.bl = pixelPts2f.bl - halfDeltaA;
        cv::Point2f halfDeltaB = (pixelPts2f.tr - pixelPts2f.br) / 55 * 35;
        pixelPts2f_Ex.br = pixelPts2f.br - halfDeltaB;
        pixelPts2f_Ex.tr = pixelPts2f.tr + halfDeltaB;

//扩展后像素坐标超过采集的图像的图幅大小
#define PTSOF(pt) (((pt).x) >= stFrameInfo.size.width || ((pt).x) < 0 || ((pt).y) >= stFrameInfo.size.height || ((pt).y) < 0)
        if (PTSOF(pixelPts2f_Ex.tl) || PTSOF(pixelPts2f_Ex.tr) || PTSOF(pixelPts2f_Ex.bl) || PTSOF(pixelPts2f_Ex.br))
#undef PTSOF
            return false;
        return true;
    }

    /**
     * 计算云台坐标系坐标
     * @change ptsInGimbal 目标在云台坐标系下的坐标
     */
    void calcWorldParams() {
        DEBUG("solvePnPRansac")
        /* 转化成相对原始图幅大小的像素坐标 */
        std::vector<cv::Point2d> gPixelPts2d;
        gPixelPts2d.reserve(4);  //灯条矩形的四个边角点坐标
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.tl) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.bl) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.br) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.tr) + cv::Point2d(stFrameInfo.offset));
        CV_Assert(!stCamera.camMat.empty());

        // 存在两种尺寸装甲板
        //如果处于小装甲板模式，使用装甲板物理参数smallFig3f；否则使用装甲板物理参数largeFig3f
        //之后对灯条矩形进行solvePnP,得出对应的旋转向量r,偏移向量t
        if (type == TARGET_SMALL)
            cv::solvePnP(stArmorStdFigure.smallFig3f, gPixelPts2d, stCamera.camMat, stCamera.distCoeffs, rv, tv);
        else
            cv::solvePnP(stArmorStdFigure.largeFig3f, gPixelPts2d, stCamera.camMat, stCamera.distCoeffs, rv, tv);

        //罗德里格斯变换，为计算角度做准备
        cv::Rodrigues(rv, rvMat);  //将旋转向量变换成旋转矩阵

        /**
         * TODO: 先将世界坐标系转到相机坐标系，再将相机坐标系变换到云台坐标系(mm) 
         * 因为需要计算的弹道，所以需要获取云台炮口的坐标
         * 
         * 我们以上步骤获得了相机坐标系的一系列参数（原点是摄像机光心）
         * 从相机坐标到云台坐标其实就是经过了一个物理空间上的位移
         * 这个位移我们可以实际测量出来，因为实际上因为相机和炮口由于不可抗的物理因素使得两者处于上下放置关系
         * 所以我们相机坐标原点到云台坐标原点只需要z坐标补偿一个相对距离（mm）
         */
        cv::Mat ptsInCamera_Mat = rvMat * stArmorStdFigure.smallShootPosition + tv;  //世界坐标系转到相机坐标系
        DEBUG("ptsInCamera_Mat")
        ptsInGimbal.x = ptsInCamera_Mat.at<double>(0, 0);
        ptsInGimbal.y = ptsInCamera_Mat.at<double>(0, 1) - 55;
        ptsInGimbal.z = ptsInCamera_Mat.at<double>(0, 2) - 25;  //云台和相机光心的垂直坐标补偿(mm)
        DEBUG("calcWorldParams end")
    }

    /**
     * 欧拉角转世界坐标点
     * @param gYaw_
     * @param gPitch_
     * @change m_rotY, m_rotX
     * @change ptsInWorld 世界坐标
     */
    void convert2WorldPts(float gYaw_, float gPitch_) {
        //单位转换，弧度转度
        gYaw_ = gYaw_ * M_PI / (180.0);
        gPitch_ = gPitch_ * M_PI / (180.0);

        /* yaw 为绕y轴旋转的 */
        m_rotY = (cv::Mat_<double>(3, 3) << std::cos(gYaw_), 0, std::sin(gYaw_),
            0, 1, 0,
            -std::sin(gYaw_), 0, std::cos(gYaw_));

        /* pitch 为绕x轴旋转的 */
        m_rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, std::cos(gPitch_), -std::sin(gPitch_),
            0, std::sin(gPitch_), std::cos(gPitch_));

        /* 先绕动系y轴旋转, 再绕动系x轴旋转 */
        cv::Mat _pts = (cv::Mat_<double>(3, 1) << ptsInGimbal.x, ptsInGimbal.y, ptsInGimbal.z);
        cv::Mat ptsInWorldMat = m_rotY * m_rotX * _pts;
        ptsInWorld.x = ptsInWorldMat.at<double>(0);
        ptsInWorld.y = ptsInWorldMat.at<double>(1);
        ptsInWorld.z = ptsInWorldMat.at<double>(2);

        DEBUG("convert2WorldPts end")
    }

    /**
     * @param cv::Point3d &v 世界坐标点
     * 世界坐标点转云台坐标点
     */
    void convert2GimbalPts(cv::Point3d &v) {
        //创建一个3x1矩阵并将物体在预测后的世界坐标系下坐标放入
        cv::Mat _pts = (cv::Mat_<double>(3, 1) << ptsInWorld_Predict.x, ptsInWorld_Predict.y, ptsInWorld_Predict.z);
        //convert2WorldPts获得的m_rotY,m_rotX取逆
        cv::Mat m_rotY_inv = m_rotY.inv();
        cv::Mat m_rotX_inv = m_rotX.inv();
        //创建一个3x1矩阵并将世界坐标点放入
        cv::Mat _v_Mat = (cv::Mat_<double>(3, 1) << v.x, v.y, v.z);
        //世界坐标->云台坐标转换
        cv::Mat ptsInGimbal_PredictMat = m_rotY_inv * m_rotX_inv * _pts;
        cv::Mat vInGimbal = m_rotY_inv * m_rotX_inv * _v_Mat;
        vInGimbal3d.x = vInGimbal.at<double>(0);
        vInGimbal3d.y = vInGimbal.at<double>(1);
        vInGimbal3d.z = vInGimbal.at<double>(2);
        ptsInGimbal_Predict.x = ptsInGimbal_PredictMat.at<double>(0);
        ptsInGimbal_Predict.y = ptsInGimbal_PredictMat.at<double>(1);
        ptsInGimbal_Predict.z = ptsInGimbal_PredictMat.at<double>(2);
    }

    /**
     */
    void correctTrajectory_and_calcEuler(float bulletSpeed) {
        /* 弹道修正, TODO */
        if(bulletSpeed){
            stCamera.correctTrajectory(ptsInGimbal,ptsInShoot,bulletSpeed);
        }
        else{
            ptsInShoot = ptsInGimbal;
        }
        DEBUG("stCamera.correctTrajectory")
        /* 计算欧拉角 */
        Camera::convertPts2Euler(ptsInShoot, &rYaw, &rPitch);  //计算Pitch,Yaw传递给电控
        DEBUG("Camera::convertPts2Euler")
    }
};  // end struct Target

/* PID控制算法 */
class PID {
  private:
    double m_p, m_i, m_d;
    double m_sum;
    double lastError;
    int64_t lastTimeStamp;
    std::atomic_bool isFirst;
    std::mutex m_mutex;

    double limit(double in, double low, double up) { return in > up ? up : (in < low ? low : in); }

  public:
    explicit PID() : m_p(0), m_i(0), m_d(0), m_sum(0), lastError(0), lastTimeStamp(0), isFirst(true) {}

    /**
     *
     * @param p 比例
     * @param i 积分
     * @param d 微分
     */
    void init(double p, double i, double d) {
        m_p = (p);
        m_i = (i);
        m_d = (d);
    }

    /**
     * @param error 
     * @param timestamp 时间戳
     * func 对ryaw进行pid控制修正
     * @return out
     */
    double calc(double error, int64_t timeStamp) {
        std::lock_guard<std::mutex> lockGuard(m_mutex);  //互斥锁
        double out = error;
        if (!isFirst) {
            if (lastError * error < 0)
                m_sum = 0;
            m_sum += error;
            m_sum = limit(m_sum, -500, 500);                      //msum>=-500
            double dt = (timeStamp - lastTimeStamp) / 1000000.0;  //s
            dt = limit(dt, 0, 0.010);                             //保证ts>=0
            assert(dt > 0);
            out = error * m_p + m_sum * m_i * dt + (error - lastError) * m_d / dt;
        } else {
            isFirst = false;
            m_sum = 0;
        }
        lastError = error;
        lastTimeStamp = timeStamp;
        return limit(out, -5, 5);  //out>=-5
    }

    bool clear() {
        return isFirst.exchange(true);
    }
};

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
        m_kf.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 1, 0,
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
}  // namespace armor

#endif  //ATTACK_BASE_HPP
