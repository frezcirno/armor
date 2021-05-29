#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

#include "debug.h"
#include "toml.hpp"

/**
 * 配置文件结构体
 */
struct Config {
    toml::Value config;

    explicit Config(const std::string &file_path) {
        /* parse config.toml */
        std::ifstream ifs(file_path);
        toml::ParseResult pr = toml::parse(ifs);
        ifs.close();
        if (!pr.valid()) {
            PRINT_ERROR("[config] config toml error: %s\n", pr.errorReason.c_str());
            PRINT_ERROR("[config] abort!\n");
            exit(0);
        }
        config = pr.value;
    }

    template <typename T>
    inline typename toml::call_traits<T>::return_type get(const std::string &key) const {
        return config.get<T>(key);
    }
} stConfig("../config.toml");

/**
 * 采集的图像的参数
 */
struct FrameInfo {
    cv::Size2i size = cv::Size2i(0, 0);      // 硬件返回的图像尺寸
    cv::Point2i offset = cv::Point2i(0, 0);  // 硬件ROI的起始点
} stFrameInfo;

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
} stCamera("../data/camera8mm.xml");

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
     * 顺序：tl,bl,br,tr
     *
     *   _(z)
     *   /'
     *  /
     * 1-----4---> (x)
     * |  x  |
     * 2-----3
     * |
     * v
     * (-y)
     */
    // 世界坐标系中，大小装甲板的坐标（**仅覆盖灯条的矩形**）
    std::vector<cv::Point3d> smallFig3f = {cv::Point3d(0, 0, 0), cv::Point3d(0, -55, 0), cv::Point3d(135, -55, 0), cv::Point3d(135, 0, 0)};
    std::vector<cv::Point3d> largeFig3f = {cv::Point3d(0, 0, 0), cv::Point3d(0, -55, 0), cv::Point3d(230, -55, 0), cv::Point3d(230, 0, 0)};
    // 扩展区域
    std::vector<cv::Point2f> smallFig_Ex = {cv::Point2f(0, 0), cv::Point2f(0, 126), cv::Point2f(135, 126), cv::Point2f(135, 0)};
    std::vector<cv::Point2f> largeFig_Ex = {cv::Point2f(0, 0), cv::Point2f(0, 126), cv::Point2f(230, 126), cv::Point2f(230, 0)};
    // 按照上面世界坐标系中，大小装甲板中心的坐标
    cv::Mat smallShootPosition = cv::Mat(cv::Point3d(67.5, -27.5, 0.0));
    cv::Mat largeShootPosition = cv::Mat(cv::Point3d(115, -27.5, 0.0));
} stArmorStdFigure;

/**
 * 灯条结构体
 */
struct Light {
    cv::Point2f topPt;
    cv::Point2f bottomPt;
    cv::Point2f centerPt;
    double angle = 0; // 灯条倾斜角，范围[0, 180)
    double length = 0;
};
