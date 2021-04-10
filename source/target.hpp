#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

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
        /* 转化成相对原始图幅大小的像素坐标 */
        std::vector<cv::Point2d> gPixelPts2d;
        gPixelPts2d.reserve(4);  //灯条矩形的四个边角点坐标
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.tl) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.bl) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.br) + cv::Point2d(stFrameInfo.offset));
        gPixelPts2d.emplace_back(cv::Point2d(pixelPts2f.tr) + cv::Point2d(stFrameInfo.offset));
        CV_Assert(!stCamera.camMat.empty());

        /** 三个坐标系：
         * 相机坐标系（3D）：
         *   原点：相机
         *   z轴：相机朝向
         *   x轴：相机朝向的右方
         *   y轴：相机朝向的下方
         * 世界/模型坐标系（3D）：
         *   自由定义
         * 图像平面（图像坐标系）（2D）：
         *   原点：图像的坐上角点
         *   x轴：向右
         *   y轴：向下
         * 
         ** solvePnP
         * solvePnP(世界/模型坐标系坐标[Pworld{0-3}], 图像坐标系坐标[Pimage{0-3}], ...)
         * 返回：旋转向量rv 和 平移矩阵tv，表示如何从世界坐标系到**相机**坐标系
         *      即，一个点在世界坐标系下的坐标为(u,v,w)，在相机坐标系下的坐标为(x,y,z)
         *      则有 [x y z]^T == R * [u v w]^T + t
         *      其中R为旋转矩阵（3x3），t为平移向量（1x3）
         **
         * 此处世界坐标系定义：O - 装甲板tl角点, x - Right, y - Up, z - Forward, 单位：mm
         * 图像坐标系为OpenCV图像坐标系
         */
        if (type == TARGET_SMALL)
            cv::solvePnP(stArmorStdFigure.smallFig3f, gPixelPts2d, stCamera.camMat, stCamera.distCoeffs, rv, tv);
        else
            cv::solvePnP(stArmorStdFigure.largeFig3f, gPixelPts2d, stCamera.camMat, stCamera.distCoeffs, rv, tv);

        /** 旋转矩阵
         * 用矩阵来表示三维旋转变换：
         *   三维旋转变换可以看成是矩阵乘法运算，绕任意轴旋转则可以分解成绕三个坐标轴旋转的叠加
         * Rx = [[1 0 0];[0 cos(a) -sin(a)];[0 sin(a) cos(a)]]
         * Ry = [[cos(a) 0 sin(a)];[0 1 0];[-sin(a) 0 cos(a)]]
         * Rz = [[cos(a) -sin(a) 0];[sin(a) cos(a) 0];[0 0 1]]
         * R = Rx * Ry * Rz
         * 特点：用9个量描述旋转变换的3个自由度，有冗余
         *      9个量是有约束的：必须是正交矩阵，且行列式为1
         *
         ** 旋转向量
         * 用一个三维向量来表示三维旋转变换，该向量的方向是旋转轴，其模则是旋转角度
         * 特点：旋转向量代表的变换关系十分直观
         *      但运算上要比矩阵形式更加复杂。
         *
         ** 相互转换
         * opencv 中有函数 Rodrigues() 用于旋转矩阵和旋转向量的转换。
         */
        cv::Rodrigues(rv, rvMat);  //将旋转向量变换成旋转矩阵

        /**
         * 得到目标点在相机坐标系下的坐标
         */
        cv::Mat ptsInCamera_Mat = rvMat * (type == TARGET_SMALL ? stArmorStdFigure.smallShootPosition : stArmorStdFigure.largeShootPosition) + tv;

        /** 目标点在（相机坐标系 -> 云台坐标系）坐标转换
         * 
         * 云台坐标系：以发射机构为中心的坐标系
         * 相机坐标系定义见上方
         */
        ptsInGimbal.x = ptsInCamera_Mat.at<double>(0, 0);
        ptsInGimbal.y = ptsInCamera_Mat.at<double>(0, 1) + 55; // 垂直方向
        ptsInGimbal.z = ptsInCamera_Mat.at<double>(0, 2) - 25; // 前后方向
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
        if (!bulletSpeed) {
            bulletSpeed = 12;
        }
        /* 弹道修正, TODO */
        stCamera.correctTrajectory(ptsInGimbal, ptsInShoot, bulletSpeed);
        /* 计算欧拉角 */
        Camera::convertPts2Euler(ptsInShoot, &rYaw, &rPitch);  //计算Pitch,Yaw传递给电控
    }
};  // end struct Target