//2021 1 26
#ifndef WINDMILL_HPP
#define WINDMILL_HPP

#include <string>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "base.hpp"
#include "imageshow.hpp"
#include "Target.hpp"
#include "Tool.hpp"
#include "opencv2/opencv.hpp"

#define WIDTH 215                          // 装甲板宽度相对值
#define HEIGHT 125                         // 装甲板高度相对值
#define RADIUS 740                         // 风车中心到边缘装甲板中心相对值(风车半径相对值)
#define ANGULAR_VELOCITY 60 / 180.0 * M_PI // 风车角速度
#define EPS 1e-2                           // 误差值

namespace wm
{
    typedef enum {TRACK, JUMP} State;//风车的状态
    //----------------------------------------------------------------------------------------------------------------------
    // 风车类
    //    为了保持接口不变，保留了之前的数据成员，但是很多没有用到
    // ---------------------------------------------------------------------------------------------------------------------
    class Windmill 
    {
    public:
    //datas
        std::string mode;      //风车模式 匀速还是三角函数运转
        State state = JUMP;    //状态值
        int stateCount = 0;    //状态计数器

        Target findedTarget;   //找到的目标 Target是目标类，详见Target.hpp
        Target hitTarget;      //预测后的击打目标

        cv::Point2f center;    //风车中心点坐标

        double timeStamp = 0.0;//相机时间戳
        double delay;          //子弹飞行延迟时间

        cv::Mat camMatrix;     //相机参数
        cv::Mat distCoeffs;    //畸变矩阵

        cv::Mat TvCtoL;        //相机到云台的变换矩阵

        float radius = RADIUS;     //风车半径
        float v = ANGULAR_VELOCITY;//旋转角速度

        float gPitch = 0.0;        //电控传来的pitch 云台与世界参照系
        float gYaw = 0.0;          //电控传来的yaw   云台与世界参照系

        int sampleNumber = 0;      //样本数目
        double spinAngle = 0.0;    //预测用到角度

        double maxPitchError = 0.0;//最大pitch误差
        double maxYawError = 0.0;  //最大yaw误差
        
        int thre;                  //二值化阈值
        bool close;                //灰度图是否进行形态学闭运算
    //fuctions
        bool run(const cv::Mat &frame, float &pitch, float &yaw, double time);
        bool detect(const cv::Mat &frame);
        void predictTarget();
        void hit(float &pitch, float &yaw);
        static Windmill *GetInstance(const cv::Mat &cam, const cv::Mat &dist,
                                     const cv::Mat &TvCtoL, const double delay, 
                                     const cv::String &modelName, ImageShowClient *is,
                                     const double maxPitchError, const double maxYawError) 
        {
            static Windmill instance(cam, dist, TvCtoL, delay, modelName, is,maxPitchError,maxYawError);
            return &instance;
        };
    private:
    //datas
        ImageShowClient *is;
    //fuctions
        explicit Windmill(const cv::Mat &cam, const cv::Mat &dist, 
                          const cv::Mat &TvCtoL, const double delay,
                          const cv::String& modelName, ImageShowClient *is,
                          const double maxPitchError, const double maxYawError)
                : camMatrix(cam), distCoeffs(dist), TvCtoL(TvCtoL), delay(delay), is(is), maxPitchError(maxPitchError), maxYawError(maxYawError) {};

    };//end of class Windmill

    //----------------------------------------------------------------------------------------------------------------------
    // 下面是一些工具函数
    // ---------------------------------------------------------------------------------------------------------------------
    // 此函数用于改变hsv阈值
    void set_hsv(int &LowH, int &LowS, int &LowV, int &HighH, int &HighS, int &HighV)
    {
        LowH = 100;
        LowS = 100;
        LowV = 90;

        HighH = 200;
        HighS = 255;
        HighV = 255;
    };
    // 此函数用于绘制旋转矩形
    void draw_rotated(const cv::Mat &mat, cv::RotatedRect &t, cv::Scalar color = cv::Scalar(0, 255, 0)) //旋转矩形绘制
    {
        cv::Point2f *vertices = new cv::Point2f[4];
        t.points(vertices);
        for (size_t i = 0; i < 4; i++)
            cv::line(mat, vertices[i], vertices[(i + 1) % 4], color, 2);
    }
    // 此函数计算向量p和向量q的叉乘
    double _dot(cv::Point2f p, cv::Point2f q)
    {
        return p.x * q.y - q.x * p.y;
    }; 
    // 此函数用于判断线段AB和线段CD是否有交点 详见博客https://www.cnblogs.com/tuyang1129/p/9390376.html
    bool _judge(cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D)
    {
        cv::Point2f AC = C - A;
        cv::Point2f AB = B - A;
        cv::Point2f AD = D - A;
        cv::Point2f CA = A - C;
        cv::Point2f CD = D - C;
        cv::Point2f CB = B - C;
        if (_dot(AB, AC) * _dot(AB, AD) >= 0)
            return false;
        if (_dot(CD, CA) * _dot(CD, CB) >= 0)
            return false;
        return true;
    };
 
    //----------------------------------------------------------------------------------------------------------------------
    // 此函数通过pitch yaw和旋转顺序计算旋转矩阵
    // 输入：pitch yaw和旋转顺序
    // 输出：旋转矩阵
    // ---------------------------------------------------------------------------------------------------------------------
    bool Windmill::run(const cv::Mat &frame, float &pitch, float &yaw, double time) 
    {
        findedTarget.clear();
        hitTarget.clear();
        timeStamp = time;
        if (detect(frame)) {//如果找到目标
            //计算从装甲片坐标系到摄像头坐标系的旋转矩阵和平移矩阵
            calRvTv(findedTarget.vertexs, WIDTH, HEIGHT, camMatrix, distCoeffs, findedTarget.RvTtoL, findedTarget.TvTtoL);
            //进行目标点的预测 需要用到上面计算的两个矩阵
            predictTarget();
            //计算pitch和yaw角度
            hit(pitch, yaw);                                                   
            return true;
        }
        STATE(tensorflow::INFO, "no target", 1)
        return false;
    };

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数检测装甲板目标并将目标存入数据成员findedTarget里
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    bool Windmill::detect(const cv::Mat &frame) 
    {
        assert(!frame.empty());

        bool isFind = false;//是否能在这一帧里找到目标
        std::vector<cv::Mat> tmp(4);//分类用装甲板模板
        tmp[0] = cv::imread("../pics/0.png", cv::IMREAD_GRAYSCALE);
        tmp[1] = cv::imread("../pics/1.png", cv::IMREAD_GRAYSCALE);
        tmp[2] = cv::imread("../pics/2.png", cv::IMREAD_GRAYSCALE);
        tmp[3] = cv::imread("../pics/3.png", cv::IMREAD_GRAYSCALE);

        int LowH, LowS, LowV, HighH, HighS, HighV;
        cv::Mat gray, hsv, mask0, mask1;
        std::vector<cv::Mat> hsvsplit;
        // 分离HSV通道
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, hsvsplit);
        // 阈值二值化
        set_hsv(LowH, LowS, LowV, HighH, HighS, HighV);
        cv::inRange(hsv, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask0);

        LowH = 0;
        HighH = 10;
        // 再次二值化
        cv::inRange(hsv, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask1);
        gray = mask0 + mask1;
        is->addImg("gray", gray);
        
        // 是否进行形态学闭运算，用于连接分断点
        if (close) {
            cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

            cv::dilate(gray, gray, element1);                        //膨胀处理
            cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, element2); //形态学闭运算
        }
        is->addImg("close", gray);

        std::vector<cv::Vec4i> hierarchy;
        std::vector<int> indexs;    // 记录外层轮廓的索引
        std::vector<int> indexs2;   // 记录内层轮廓的索引  
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        cv::RotatedRect Rrect;   //扇叶
        cv::RotatedRect Armor;   //装甲板
        
        double dis;              //装甲板中心到风车中心的距离
        if (hierarchy.size())
            for (int i = 0; i >= 0; i = hierarchy[i][0]) //遍历轮廓
            {
                if (contours[i].size() < 5)
                    continue; //椭圆拟合至少五个点

                Rrect = cv::fitEllipse(contours[i]);
                is->addText(cv::format("windmill-fan-size: %f", Rrect.size.area()));
                if (Rrect.size.area() > 200 && Rrect.size.area() < 1000 && Rrect.size.height / Rrect.size.width < 1.4)//筛选中心发光R
                {
                    is->addText("find center!");
                   
                    std::vector<cv::RotatedRect> Rrect_list1;
                    Rrect_list1.push_back(Rrect);
                    is->addRotatedRects("find center!", Rrect_list1);
                    draw_rotated(frame, Rrect, cv::Scalar(0, 255, 255));//draw
                    is->addText(cv::format("windmill-center-size: %f", Rrect.size.area()));
                    std::cout << "*******************" << Rrect.size.area() << "**************" <<std:: endl;
                    center = Rrect.center;
                } //找到风车的中心

                if (Rrect.size.area() < 200 || Rrect.size.area() > 60000)
                    continue;

                if (Rrect.size.height > frame.rows || Rrect.size.width > frame.cols)
                    continue;
                
                //draw_rotated(frame, Rrect);//draw
                std::vector<cv::RotatedRect> Rrect_list2;
                Rrect_list2.push_back(Rrect);
               // is->addRotatedRects("windmill-fan", Rrect_list2);
                indexs.push_back(i);
            }
        
        is->addText(cv::format("windmill-center-x: %f center-y: %f", center.x, center.y));
        
        // Rrect是扇叶
        // 从opencv的接口来看，生成RotatedRect的函数有两个，minAreaRect和fitEllipse，前者angle的范围是-90到0，后者范围从0到180
        // 详见博客https://blog.csdn.net/qq_34793133/article/details/82497996
        if (indexs.size())
            for (int t = 0; t < indexs.size(); t++) {//遍历轮廓
                int i = indexs[t];

                if (contours[i].size() < 5)
                    continue; //椭圆拟合至少五个点
                Rrect = cv::fitEllipse(contours[i]);
                int outer_long = std::max(Rrect.size.width, Rrect.size.height);
                int outer_short = std::min(Rrect.size.width, Rrect.size.height);

                cv::Point2f p[4], srcRect[4], dstRect[4];
                Rrect.points(p);

                srcRect[0] = p[0];
                srcRect[1] = p[1];
                srcRect[2] = p[2];
                srcRect[3] = p[3];
                
                dstRect[0] = cv::Point2f(0, 0);
                dstRect[1] = cv::Point2f(Rrect.size.height, 0);
                dstRect[2] = cv::Point2f(Rrect.size.height, Rrect.size.width);
                dstRect[3] = cv::Point2f(0, Rrect.size.width);

                cv::Mat transform = cv::getPerspectiveTransform(srcRect, dstRect);
                /* 由四对点计算透射变换
                    CvMat* cvGetPerspectiveTransform(const CvPoint2D32f* src, const CvPoint2D32f* dst, CvMat*map_matrix);
                    src 输入图像的四边形顶点坐标。
                    dst 输出图像的相应的四边形顶点坐标。
                    map_matrix 指向3×3输出矩阵的指针。
                */
                cv::Mat perspectMat;
                cv::warpPerspective(gray, perspectMat, transform, gray.size());
                /* 对图像进行透视变换
                    void cvWarpPerspective(const CvArr* src, CvArr* dst, const CvMat* map_matrix, 
                                        int flags = CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, CvScalar fillval = cvScalarAll(0) )
                    src 输入图像
                    dst 输出图像
                    map_matrix 3×3 变换矩阵
                */
                cv::Mat right;
                right = perspectMat(cv::Rect(0, 0, Rrect.size.height, Rrect.size.width)); //透视变换后的旋转臂图
                cv::resize(right, right, cv::Size(325, 155));

                cv::Mat result;
                double tmax;
                double tmin;
                double vmax = -2;
                int index = -1;

                for (int j = 0; j < 4; ++j)//模板匹配，得分最高的为所属类
                {
                    cv::matchTemplate(right, tmp[j], result, cv::TM_CCOEFF_NORMED);
                    cv::minMaxLoc(result, &tmin, &tmax);
                    if (tmax > vmax) {
                        vmax = tmax;
                        index = j;
                    }
                }

                if (index < 2)//索引小于2,说明是待击打旋转臂
                {
                    int a_i = hierarchy[i][2];//子轮廓，找到装甲板
                    for (int b_i = a_i; b_i >= 0; b_i = hierarchy[b_i][0]) {//筛选装甲板，通过长宽比和装甲板宽度和旋转臂宽度比较筛选
                        if (contours[b_i].size() < 5)
                            continue; //椭圆拟合至少五个点
                        Armor = cv::fitEllipse(contours[b_i]);
                        int inner_long = std::max(Armor.size.width, Armor.size.height);
                        int inner_short = std::min(Armor.size.width, Armor.size.height);

                        std::cout << "outer_long-outer_short-inner_long-inner_short: " << outer_long << " " << outer_short << " " << inner_long << " " << inner_short << std::endl;
                        
                        if (abs(inner_long - outer_short) / double(outer_short) < 0.5 
                          && double(inner_long) / inner_short < 3) {
                            
                            std::cout << "outer_long-outer_short-inner_long-inner_short: " << outer_long << " " << outer_short << " " << inner_long << " " << inner_short << std::endl;
                            
                            break;
                        }
                    }
                        
                   // draw_rotated(frame, Armor, cv::Scalar(255, 255, 0));//draw
                    std::vector<cv::RotatedRect> Rrect_list3;
                    Rrect_list3.push_back(Armor);
                    //is->addRotatedRects("windmill-armor", Rrect_list3);

                    dis = sqrt((Armor.center.x - center.x) * (Armor.center.x - center.x) + 
                                     (Armor.center.y - center.y) * (Armor.center.y - center.y));

                    this->radius = dis;//给半径赋值

                    //给4个顶点排序
                    //因为要solvePnP进行打击点的预测，所以顺序非常重要
                    /*+———————————————————————+
                      |                       |
                      |                       |
                      |                       |
                      +———————————————————————+*/
                    //装甲板坐标系定义：原点是左下角 横轴是x轴 纵轴是y轴 
                    //依次对应的四个点：左上角 左下角 右下角 右上角
                    //   排序思路，首先找到内边和外边，然后确定原点，可以通过 连接装甲板中心点和风车中心点的线段 
                    //以及 连接装甲板边的线段是否有交点来进行判断，后续进一步确定原点可以通过旋转矩形四个顶点按照
                    //顺时针排序思路进行。
                    //本身points函数是按照顺时针排列的
                    cv::Point2f temp[4];
                    std::vector<cv::Point2f> final;
                    Armor.points(temp);

                    if (_judge(temp[0], temp[1], center, Armor.center)) {
                        isFind = true;
                        final.push_back(temp[2]);
                        final.push_back(temp[1]);
                        final.push_back(temp[0]);
                        final.push_back(temp[3]);
                    }  else if (_judge(temp[1], temp[2], center, Armor.center)) {
                        isFind = true;
                        final.push_back(temp[3]);
                        final.push_back(temp[2]);
                        final.push_back(temp[1]);
                        final.push_back(temp[0]);
                    } else if (_judge(temp[2], temp[3], center, Armor.center)) {
                        isFind = true;
                        final.push_back(temp[0]);
                        final.push_back(temp[3]);
                        final.push_back(temp[2]);
                        final.push_back(temp[1]);
                    } else if (_judge(temp[3], temp[0], center, Armor.center)) {
                        isFind = true; 
                        final.push_back(temp[1]);
                        final.push_back(temp[0]);
                        final.push_back(temp[3]);
                        final.push_back(temp[2]);
                    } 
                    else {
                        isFind = false;

                        final.push_back(temp[0]);
                        final.push_back(temp[0]);
                        final.push_back(temp[0]);
                        final.push_back(temp[0]);
                    }

                    //cv::circle(frame, final[0], 10, {0, 255, 0});
                    //cv::circle(frame, final[1], 20, {0, 255, 0});
                    //cv::circle(frame, final[2], 30, {0, 255, 0});
                   // cv::circle(frame, final[3], 40, {0, 255, 0}); //点位判断
                    findedTarget.vertexs = final;
                }
            }
        
        is->addImg("frame", frame, true);
        return isFind;
    };

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数计算要打击的目标点的世界坐标系坐标
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    void Windmill::predictTarget() {
        //预测子弹飞行过程中扇叶旋转的角度 delay为飞行的时间 v是飞行的速度
        float theta = this->delay * this->v;
        this->hitTarget = this->findedTarget;
        //预测的目标点在装甲板坐标系中的坐标
        (this->hitTarget).tPt = cv::Point3f(sin(theta) * this->radius + WIDTH / 2, -(this->radius - cos(theta) * this->radius - HEIGHT / 2), 0.0);
        (this->hitTarget).vertexs.clear();
        
        cv::Point2f outputPt;
        cv::Point2f detectedPt;
        //将击打的目标点从三维装甲板坐标系转换到二维图片坐标系
        coodiTrans(hitTarget.tPt, outputPt, 
                   hitTarget.RvTtoL, hitTarget.TvTtoL,
                   camMatrix, distCoeffs);
        //将检测的目标点从三维装甲板坐标系转换到二维图片坐标系
        coodiTrans(cv::Point3f(WIDTH / 2, HEIGHT / 2, 0.0), detectedPt, 
                   hitTarget.RvTtoL, hitTarget.TvTtoL,
                   camMatrix, distCoeffs);

        //要击打的目标
        hitTarget.vertexs.emplace_back(outputPt.x - 5, outputPt.y + 5);
        hitTarget.vertexs.emplace_back(outputPt.x - 5, outputPt.y - 5);
        hitTarget.vertexs.emplace_back(outputPt.x + 5, outputPt.y - 5);
        hitTarget.vertexs.emplace_back(outputPt.x + 5, outputPt.y + 5);

        //检测到的目标
        std::vector<cv::Point2f> detectedPts;
        detectedPts.emplace_back(detectedPt.x - 5, detectedPt.y + 5);
        detectedPts.emplace_back(detectedPt.x - 5, detectedPt.y - 5);
        detectedPts.emplace_back(detectedPt.x + 5, detectedPt.y - 5);
        detectedPts.emplace_back(detectedPt.x + 5, detectedPt.y + 5);

        is->addEvent("detected target", detectedPts);
        return;
    };

    //----------------------------------------------------------------------------------------------------------------------
    // 此函数计算要返回给电控的pitch和yaw角度
    // 输入：
    // 输出：
    // ---------------------------------------------------------------------------------------------------------------------
    void Windmill::hit(float &pitch, float &yaw) 
    {
        //计算打击目标点在摄像头坐标系下的坐标
        coodiTrans(hitTarget.tPt, hitTarget.lPt, hitTarget.RvTtoL, hitTarget.TvTtoL);
        cv::Mat eye = cv::Mat::eye(3, 3, CV_64FC1);
        //计算打击目标点在云台坐标系下的坐标
        coodiTrans(hitTarget.lPt, hitTarget.lPt, eye, this->TvCtoL);
        //计算打击目标点的pitch和yaw角度
        calPitchYaw(hitTarget.lPt, pitch, yaw);
        
        is->addText(cv::format("pitch: %0.2f", pitch));
        is->addText(cv::format("yaw: %0.2f", yaw));
        
        if (delay > 0) {//顺时针旋转
            //pitch -= maxPitchError * cos(spinAngle / 180.0 * M_PI);
            //yaw += maxYawError * sin(spinAngle / 180 * M_PI);
        }
        else if (delay < 0) {//逆时针旋转
            //pitch += maxPitchError * cos(spinAngle / 180.0 * M_PI);
            //yaw -= maxYawError * sin(spinAngle / 180 * M_PI);
        }
        else;

        if (this->state == JUMP && !(pitch < 2.0 && yaw < 2.0)) 
            this->state = JUMP;
        else if (this->state == JUMP && pitch < 2.0 && yaw < 2.0) 
            this->state = TRACK;
        else if (this->state == TRACK && pitch < 2.0 && yaw < 2.0) {
            this->stateCount = 0;
            this->state = TRACK;
        }
        else if (this->state == TRACK && !(pitch < 2.0 && yaw < 2.0)){
            (this->stateCount)++;
            if (this->stateCount >= 3) 
                this->state = JUMP;
            else 
                pitch = yaw = 0.0;
        }
        else 
            abort();    
    };
}
#endif