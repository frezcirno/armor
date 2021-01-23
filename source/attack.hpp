#ifndef TJSP_ATTACK_2020_ATTACK_HPP
#define TJSP_ATTACK_2020_ATTACK_HPP

#include "numeric"
#include "base.hpp"
#include "capture.hpp"
#include "imageshow.hpp"
#include "communicator.hpp"
#include "ThreadPool.h"
#include "thread"
#include "future"
#include "layers.hpp"
#include "utility"
#include "dirent.h"

#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/framework/tensor.h"
#include "google/protobuf/wrappers.pb.h"
#include "tensorflow/core/util/command_line_flags.h"

using tensorflow::string;
using tensorflow::Tensor;

using namespace tensorflow;

/*模型路径*/
const string model_path = "../Model/happyModel.pb";
/*输入输出节点详见ipynb的summary*/
const string input_name = "input_1:0";
const string output_name = "y/Sigmoid:0";

namespace armor
{
/*
  自瞄基类, 多线程共享变量用
 */
    class AttackBase
    {
    protected:
        static std::mutex s_mutex;                     // 互斥锁
        static std::atomic<int64_t> s_latestTimeStamp; // 已经发送的帧编号
        static std::deque<Target> s_historyTargets;    // 打击历史, 最新的在头部, [0, 1, 2, 3, ....]
        static Kalman kalman;                          // 卡尔曼滤波
    };
    std::mutex AttackBase::s_mutex;
    std::atomic<int64_t> AttackBase::s_latestTimeStamp(0);
    std::deque<Target> AttackBase::s_historyTargets;
    Kalman AttackBase::kalman;
/*
  自瞄主类
 */
    class Attack : AttackBase
    {
    private:
        Communicator &m_communicator;
        ImageShowClient &m_is;
        cv::Mat m_bgr;
        cv::Mat m_bgr_raw;
        // 目标
        std::vector<Target> m_preTargets; // 预检测目标
        std::vector<Target> m_targets;    // 本次有效目标集合
        // 开小图
        cv::Point2i m_startPt;
        bool m_isEnablePredict;     // 是否开预测

        int64_t m_currentTimeStamp; // 当前时间戳
        PID &m_pid;                 // PID
        bool m_isUseDialte;         // 是否膨胀
        bool mode;                  // 红蓝模式

    public:
        explicit Attack(Communicator &communicator, PID &pid, ImageShowClient &isClient) : m_communicator(communicator),
                                                                                           m_is(isClient),
                                                                                           m_isEnablePredict(true), m_currentTimeStamp(0), m_pid(pid), m_isUseDialte(false)
        {
            mycnn::loadWeights("../info/dumpe2.nnet");
            m_isUseDialte = stConfig.get<bool>("auto.is-dilate");
        }
        void setMode(bool colorMode) { mode = colorMode; }

    private:
    /**
     * @name m_preDetect
     * @func 通过hsv筛选和进行预处理获得装甲板
     */
        void m_preDetect()
        {
            DEBUG("m_preDetect")
            /* 使用inRange对颜色进行筛选 */
            cv::Mat bgrChecked;
            m_is.clock("inRange");
            if (mode)
            {
                /* 红色 */
                cv::inRange(m_bgr, cv::Scalar(0, 0, 140), cv::Scalar(180, 255, 255), bgrChecked);
            }
            else
            {
                /* 蓝色 */
                cv::inRange(m_bgr, cv::Scalar(150, 200, 150), cv::Scalar(255, 255, 255), bgrChecked);
            }
            m_is.clock("inRange");
            DEBUG("inRange end")
            /* 进行膨胀操作（默认关闭） */
            m_is.addImg("bgrChecked", bgrChecked, true);
            if (m_isUseDialte)
            {
                cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                dilate(bgrChecked, bgrChecked, element);
                m_is.addImg("dilate", bgrChecked, true);
            }
            /* 寻找边缘，并圈出countours */
            std::vector<std::vector<cv::Point2i>> contours;
            cv::findContours(bgrChecked, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            m_is.addEvent("contours", contours);
            DEBUG("findContours end")

            /* 对灯条进行筛选 */
            std::vector<Light> lights;
            for (const auto &_pts : contours)
            {
                /* 设定最小面积>=5 */
                if (_pts.size() < 5)
                    continue;
                /* 寻找最小外接矩形 */
                cv::RotatedRect rRect = cv::minAreaRect(_pts);
                /* 设定长宽比2/3～3/2 */
                double hw = rRect.size.height / rRect.size.width;
                if (0.6667 < hw && hw < 1.5)
                    continue;
                /* 寻找灯条的顶部中点，底部中点与倾斜角 */
                Light _light;
                cv::Point2f topPt;      //顶部中点
                cv::Point2f bottomPt;   //底部中点
                cv::Point2f pts[4];

                rRect.points(pts);
                if (rRect.size.width > rRect.size.height)//根据外接矩形的特性需调整点
                {
                    bottomPt = (pts[2] + pts[3]) / 2.0;
                    topPt = (pts[0] + pts[1]) / 2.0;
                    _light.angle = cv::abs(rRect.angle);
                }
                else
                {
                    bottomPt = (pts[1] + pts[2]) / 2;
                    topPt = (pts[0] + pts[3]) / 2;
                    _light.angle = cv::abs(rRect.angle - 90);
                }
                /* 判断顶部和底部中点是否设置正确，并将中心点与长度一并写入_light参数中 */
                if (topPt.y > bottomPt.y)
                {
                    _light.topPt = bottomPt;
                    _light.bottomPt = topPt;
                }
                else
                {
                    _light.topPt = topPt;
                    _light.bottomPt = bottomPt;
                }
                _light.centerPt = rRect.center;             //中心点
                _light.length = cv::norm(bottomPt - topPt); //长度
                 /* 判断长度和倾斜角是否合乎要求 */
                if (_light.length < 3.0 || 800.0 < _light.length || cv::abs(_light.angle - 90) > 30.0)
                    continue;
                lights.emplace_back(_light);
            }
            DEBUG("lights end")
            m_is.addEvent("lights", lights);
            /* 对筛选出的灯条按x大小进行排序 */
            std::sort(lights.begin(), lights.end(), [](Light &a_, Light &b_) -> bool {
                return a_.centerPt.x < b_.centerPt.x;
            });
            /* 对灯条进行两两组合并筛选出预检测的装甲板 */
            for (size_t i = 0; i < lights.size(); ++i)
            {
                for (size_t j = i + 1; j < lights.size(); ++j)
                {
                    cv::Point2f AC2BC = lights[j].centerPt - lights[i].centerPt;
                    double minLength = cv::min(lights[i].length, lights[j].length);
                    double deltaAngle = cv::abs(lights[i].angle - lights[j].angle);
                    /* 对灯条组的长度，角度差，中心点tan值，x位置等进行筛选 */
                    if ((deltaAngle > 23.0 && minLength < 20) || (deltaAngle > 11.0 && minLength >= 20) ||
                        cv::abs(lights[i].length - lights[j].length) / minLength > 0.5 ||
                        cv::fastAtan2(cv::abs(AC2BC.y), cv::abs(AC2BC.x)) > 25.0 ||
                        AC2BC.x / minLength > 7.1)
                        continue;
                    Target target;
                    /* 计算像素坐标 */
                    target.setPixelPts(lights[i].topPt, lights[i].bottomPt, lights[j].bottomPt, lights[j].topPt,
                                       m_startPt);
                    if (cv::norm(AC2BC) / minLength > 4.9)
                        target.type = TARGET_LARGE; // 大装甲
                    /* 获得扩展区域像素坐标, 若无法扩展则放弃该目标 */
                    if (!target.convert2ExternalPts2f())
                        continue;
                    m_preTargets.emplace_back(target);
                }
            }
            m_is.addEvent("preTargets", m_preTargets);
            DEBUG("preTargets end")
        }
        int m_cropNameCounter = 0;

        /**
         * @name mat2Tensor
         * @param image 图片
         * @param t tensor
         * @func 将图片从mat转化为tensor
         */ 
        void mat2Tensor(cv::Mat &image, Tensor &t)
        {
            float *tensor_data_ptr = t.flat<float>().data();
            cv::Mat fake_mat(image.rows, image.cols, CV_32FC(image.channels()), tensor_data_ptr);
            image.convertTo(fake_mat, CV_32FC(image.channels()));
        }
        /**
         * @name getThreshold
         * @param mat 图片
         * @param thre_proportion 比例阈值 0.1
         * @func 得到二值化阈值
         * @return i 二值化阈值
         */ 
        int getThreshold(const cv::Mat &mat, double thre_proportion = 0.1)
        {
            /* 计算总像素数目 */
            uint32_t iter_rows = mat.rows;
            uint32_t iter_cols = mat.cols;
            auto sum_pixel = iter_rows * iter_cols;
            /* 判断是否连续*/
            if (mat.isContinuous())
            {
                iter_cols = sum_pixel;
                iter_rows = 1;
            }
            /* 新建数组置零 */
            int histogram[256];
            memset(histogram, 0, sizeof(histogram));
             /* 像素排序 */
            for (uint32_t i = 0; i < iter_rows; ++i)
            {
                const auto *lhs = mat.ptr<uchar>(i);
                for (uint32_t j = 0; j < iter_cols; ++j)
                    ++histogram[*lhs++];
            }
            auto left = thre_proportion * sum_pixel;
            int i = 255;
            while ((left -= histogram[i--]) > 0)
                ;
            return i > 0 ? i : 0;
        }
        /**
         * @name loadAndPre
         * @param img 图片
         * @param result
         * @func 进行图片的预处理和高光补偿
         * @return true/false
         */ 
        bool loadAndPre(cv::Mat img, cv::Mat &result)
        {
            if (img.cols == 0)
                return false;
            /* 调整大小 同比缩放至fixedsize*fixedsize以内 */
            if (img.cols < img.rows)
                resize(img, img, {int(img.cols * 1.0 / img.rows * fixedSize), fixedSize});
            else
                resize(img, img, {fixedSize, int(img.rows * 1.0 / img.cols * fixedSize)});
            /* 剪去边上多余部分 */
            int cutRatio1 = 0.15 * img.cols;
            int cutRatio2 = 0.05 * img.rows;
            cv::Mat blank = cv::Mat(cv::Size(fixedSize, fixedSize), img.type(), cv::Scalar(0));                           //新建空白
            cv::Mat mask = img(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2));       //建立腌摸
            cv::Mat imageROI = blank(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2)); //建立需要覆盖区域的ROI
            mask.copyTo(imageROI, mask);
            int thre = getThreshold(blank);   //均值获取阈值
            result = blank.clone();
            /* 使用二值化阈值补高光 */
            for (int i = 0; i < result.rows; i++)
            {
                for (int j = 0; j < result.cols; j++)
                {
                    if ((int)result.at<u_char>(i, j) > thre)
                        result.at<u_char>(i, j) = 200;    
                }
            }
            return true;
        }
        /**
         * @name init_my_tf
         * @param session 交互接口
         * @func 读取模型并设置到session中
         * @return input
         */ 
        inline Tensor init_my_tf(Session *session)
        {
            /* 从pb文件中读取模型 */
            GraphDef graph_def;
            Status status = ReadBinaryProto(Env::Default(), model_path, &graph_def);  //读取Graph, 如果是文本形式的pb,使用ReadTextProto
            if (!status.ok())
                std::cout << status.ToString() << std::endl;
            else
                std::cout << "Load graph protobuf successfully" << std::endl;
            /* 将模型设置到创建的Session里 */
            status = session->Create(graph_def);
            if (!status.ok())
                std::cout << status.ToString() << std::endl;
            else
                std::cout << "Add graph to session successfully" << std::endl;
            Tensor input(DT_FLOAT, TensorShape({1, fixedSize, fixedSize, 1}));
            return input;
        }
        /**
         * @name m_classify_single_tensor
         * @param isSave 是否保存样本图片
         * @func 基于tensorflow的分类器
         */ 
        void m_classify_single_tensor(bool isSave = false)
        {
            if (m_preTargets.empty())
                return;
            Session *session;
            /* 创建session */
            Status status = NewSession(SessionOptions(), &session);
            if (!status.ok())
                std::cout << status.ToString() << std::endl;
            else
                std::cout << "Session created successfully" << std::endl;
            /* 初始化session */
            Tensor input = init_my_tf(session);

            for (auto &_tar : m_preTargets)
            {
                cv::Rect tmp = cv::boundingRect(_tar.pixelPts2f_Ex);
                cv::Mat tmp2 = m_bgr_raw(tmp).clone();
                /* 将图片变成目标大小 */
                cv::Mat transMat = cv::getPerspectiveTransform(_tar.pixelPts2f_Ex,
                                                               _tar.pixelPts2f_Ex);
                cv::Mat _crop;
                /* 投影变换 */
                cv::warpPerspective(tmp2, _crop, transMat, cv::Size(tmp2.size())); 
                /* 转灰度图 */
                cv::cvtColor(_crop, _crop, cv::COLOR_BGR2GRAY);
                /* 储存图 */
                if (isSave)
                {
                    cv::imwrite(cv::format("../data/raw/%d.png", m_cropNameCounter++), _crop);
                }
                cv::Mat image;
                if (loadAndPre(_crop, image))
                {
                    /* mat转换为tensor */
                    mat2Tensor(image, input);
                    /* 保留最终输出 */
                    std::vector<tensorflow::Tensor> outputs;
                    /* 计算最后结果 */
                    TF_CHECK_OK(session->Run({std::pair<string, Tensor>(input_name, input)}, {output_name}, {}, &outputs));
                    /* 获取输出 */
                    auto output_c = outputs[0].scalar<float>();
                    float result = output_c();
                    /* 判断正负样本 */
                    if (0.5 < result)
                        m_targets.emplace_back(_tar);
                }
                else
                    continue;
            }
            session->Close();
            m_is.addClassifiedTargets("After Classify", m_targets);
            DEBUG("m_classify end")
        }
        /**
         * @name m_match
         * @func 击打策略函数
         */ 
        emSendStatusA m_match()
        {
            /* 更新下相对帧编号 */
            for (auto iter = s_historyTargets.begin(); iter != s_historyTargets.end(); iter++)
            {
                iter->rTick++;
                /* 历史值数量大于30便删除末尾记录 */
                if (iter->rTick > 30)
                {
                    s_historyTargets.erase(iter, s_historyTargets.end());
                    break;
                }
            }
            /* 选择本次打击目标 */
            if (s_historyTargets.empty())
            {
                /* case A: 之前没选择过打击目标 */
                /* 选择数组中距离最近的目标作为击打目标 */
                auto minTarElement = std::min_element(
                    m_targets.begin(), m_targets.end(), [](Target &a_, Target &b_) -> bool {
                        return cv::norm(a_.ptsInGimbal) < cv::norm(b_.ptsInGimbal);
                    });                                   //找到含最小元素的目标位置
                if (minTarElement != m_targets.end())
                {
                    s_historyTargets.emplace_front(*minTarElement);
                    PRINT_INFO("++++++++++++++++ 发现目标: 选择最近的 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM;          //瞄准
                }
                else
                {
                    return SEND_STATUS_AUTO_NOT_FOUND;    //未找到
                }
            } // end case A
            else
            {
                /* case B: 之前选过打击目标了, 得找到一样的目标 */
                PRINT_INFO("++++++++++++++++ 开始寻找上一次目标 ++++++++++++++++++++\n");
                double distance = 0xffffffff;
                int closestElementIndex = -1;
                for (size_t i = 0; i < m_targets.size(); ++i)
                {
                    /* 进行轮廓匹配，所得为0～1，数值越小越好*/
                    double distanceA = cv::matchShapes(m_targets[i].pixelPts2f, s_historyTargets[0].pixelPts2f,
                                                       cv::CONTOURS_MATCH_I3, 0.0);
                    /* 获取图像矩 */                                
                    cv::Moments m_1 = cv::moments(m_targets[i].pixelPts2f);
                    cv::Moments m_2 = cv::moments(s_historyTargets[0].pixelPts2f);   
                    PRINT_WARN("distanceA = %f\n", distanceA);
                    /* 进行matchShaoes的阈值限定，并保证归一化中心矩同号 */ 
                    if (distanceA > 0.5 ||
                        (m_1.nu11 + m_1.nu30 + m_1.nu12) * (m_2.nu11 + m_2.nu30 + m_2.nu12) < 0)
                        continue;

                    double distanceB;
                    if (m_isEnablePredict)
                    {
                        /* 用绝对坐标距离计算 两次位置之差 */
                        distanceB = cv::norm(m_targets[i].ptsInWorld - s_historyTargets[0].ptsInWorld) / 2000.0;
                        PRINT_WARN("distanceB = %f\n", distanceB);
                        /* 进行阈值判定 */
                        if (distanceB > 0.5)
                            continue;
                    }
                    else
                    {
                        /* 用云台坐标系距离计算 两次位置之差 */
                        distanceB = cv::norm(m_targets[i].ptsInGimbal - s_historyTargets[0].ptsInGimbal) / 3400.0;
                        PRINT_WARN("distanceB = %f\n", distanceB);
                        /* 进行阈值判定 */
                        if (distanceB > 0.8)
                            continue;
                    }
                    double _distanceTemp = distanceA + distanceB / 2; 
                    /* 参数更正，保证当前图片存在 */
                    if (distance > _distanceTemp)
                    {
                        distance = _distanceTemp;
                        closestElementIndex = i;
                    }
                }
                if (closestElementIndex != -1)
                {
                    /* 找到了 */
                    s_historyTargets.emplace_front(m_targets[closestElementIndex]);
                    PRINT_INFO("++++++++++++++++ 找到上一次目标 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM;           //瞄准
                }
                else
                {
                    PRINT_INFO("++++++++++++++++ 没找到上一次目标, 按上一次的来 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM_FORMER;    //瞄准上一帧
                }
            } // end case B
            PRINT_ERROR("Something is NOT Handled in function m_match \n");
        }

    public:
        /**
         * @name enablePredict
         * @param enable = true: 开启
         * @func 设置是否开启预测
         */
        void enablePredict(bool enable = true)
        {
            m_communicator.enableReceiveGlobalAngle(enable);
            m_isEnablePredict = enable;
        }

        /**
         * @name getBoundingRect
         * @param tar 上一个检测到的装甲
         * @param rect 截的图
         * @param size 采集图像参数
         * @param extendFlag 是否扩展
         * @func 图像扩展ROI
         */
        void getBoundingRect(Target &tar, cv::Rect &rect, cv::Size &size, bool extendFlag = false)
        {
            rect = cv::boundingRect(s_historyTargets[0].pixelPts2f_Ex);

            if (extendFlag)
            {
                rect.x -= int(rect.width * 4);
                rect.y -= rect.height * 3;
                rect.width *= 9;
                rect.height *= 7;

                rect.width = rect.width >= size.width ? size.width - 1 : rect.width;
                rect.height = rect.height >= size.height ? size.height - 1 : rect.height;

                rect.width = rect.width < 80 ? 80 : rect.width;
                rect.height = rect.height < 50 ? 50 : rect.height;

                rect.x = rect.x < 1 ? 1 : rect.x;
                rect.y = rect.y < 1 ? 1 : rect.y;

                rect.width = rect.x + rect.width >= size.width ? size.width - 1 - rect.x : rect.width;
                rect.height = rect.y + rect.height >= size.height ? size.height - 1 - rect.y : rect.height;
            }
        }

        /**
         * @name run
         * @param src 彩图
         * @param timeStamp 时间戳
         * @param gYaw 从电控获得yaw
         * @param gPitch 从电控获得pitch
         * @func 主运行函数
         * @return true
         */
        bool run(cv::Mat &src, int64_t timeStamp, float gYaw, float gPitch)
        {
            /* 1.初始化参数，判断是否启用ROI */
            m_bgr_raw = src;
            m_bgr = src;
            m_currentTimeStamp = timeStamp;
            m_targets.clear();
            m_preTargets.clear();
            m_startPt = cv::Point(0, 0);
            if (s_historyTargets.size() >= 2 && s_historyTargets[0].rTick <= 10)
            {
                cv::Rect latestShootRect;
                getBoundingRect(s_historyTargets[0], latestShootRect, stFrameInfo.size, true);
                m_is.addEvent("Bounding Rect", latestShootRect);
                m_bgr = m_bgr(latestShootRect);
                m_startPt = latestShootRect.tl();
            }

            /* 2.预检测 */
            m_preDetect();

            /* 3.通过分类器 */
            m_is.clock("m_classify");
            m_classify_single_tensor(0); 
            m_is.clock("m_classify");

            /* 已经有更新的一帧发出去了 */
            if (timeStamp < s_latestTimeStamp.load())
                return false;

            /* 处理多线程新旧数据处理的问题 */
            std::unique_lock<std::mutex> preLock(s_mutex, std::try_to_lock);
            while (!preLock.owns_lock() && timeStamp > s_latestTimeStamp.load())
            {
                armor::thread_sleep_us(5);
                preLock.try_lock();
            }

            /* 目标匹配 + 预测 + 修正弹道 + 计算欧拉角 + 射击策略 */
            if (preLock.owns_lock() && timeStamp > s_latestTimeStamp.load())
            {
                s_latestTimeStamp.exchange(timeStamp);
                float rYaw = 0.0;
                float rPitch = 0.0;
                /* 获得云台全局欧拉角 */
                m_communicator.getGlobalAngle(&gYaw, &gPitch);
                /* 计算世界坐标参数，转换到世界坐标系 */
                for (auto &tar : m_targets)
                {
                    tar.calcWorldParams();
                    tar.convert2WorldPts(-gYaw, gPitch);
                }
                /* 4.目标匹配 */
                emSendStatusA statusA = m_match();
                DEBUG("m_match end")

                if (!s_historyTargets.empty())
                {
                    m_is.addFinalTargets("selected", s_historyTargets[0]);
                    /* 5.预测部分 */
                    if (m_isEnablePredict)
                    {
                        cout << "m_isEnablePredict start !" << endl;

                        if (statusA == SEND_STATUS_AUTO_AIM)
                        {   /* 获取世界坐标点 */
                            m_communicator.getGlobalAngle(&gYaw, &gPitch);
                            s_historyTargets[0].convert2WorldPts(-gYaw, gPitch);
                            cout << "s_historyTargets[0].ptsInGimbal : " << s_historyTargets[0].ptsInGimbal << endl;
                            /* 卡尔曼滤波初始化/参数修正 */
                            if (s_historyTargets.size() == 1)
                                kalman.clear_and_init(s_historyTargets[0].ptsInWorld, timeStamp);
                            else
                            {
                                kalman.correct(s_historyTargets[0].ptsInWorld, timeStamp);
                            }
                        }
                        m_is.addText(cv::format("inWorld.x %.0f", s_historyTargets[0].ptsInWorld.x));
                        m_is.addText(cv::format("inWorld.y %.0f", s_historyTargets[0].ptsInWorld.y));
                        m_is.addText(cv::format("inWorld.z %.0f", s_historyTargets[0].ptsInWorld.z));
                        /* 进行预测和坐标修正 */
                        if (s_historyTargets.size() > 1)
                        {
                            kalman.predict(0.1, s_historyTargets[0].ptsInWorld_Predict);
                            /* 转换为云台坐标点 */
                            s_historyTargets[0].convert2GimbalPts(kalman.velocity);
                            m_is.addText(cv::format("vx %4.0f", s_historyTargets[0].vInGimbal3d.x));
                            m_is.addText(cv::format("vy %4.0f", cv::abs(s_historyTargets[0].vInGimbal3d.y)));
                            m_is.addText(cv::format("vz %4.0f", cv::abs(s_historyTargets[0].vInGimbal3d.z)));
                            if (cv::abs(s_historyTargets[0].vInGimbal3d.x) > 1.6)
                            {
                                double deltaX = cv::abs(13 * cv::abs(s_historyTargets[0].vInGimbal3d.x) *
                                                        s_historyTargets[0].ptsInGimbal.z / 3000);
                                deltaX = deltaX > 300 ? 300 : deltaX;
                                s_historyTargets[0].ptsInGimbal.x +=
                                    deltaX * cv::abs(s_historyTargets[0].vInGimbal3d.x) /
                                    s_historyTargets[0].vInGimbal3d.x;
                            }
                        }
                    }

                    /* 6.修正弹道并计算欧拉角 */
                    DEBUG("correctTrajectory_and_calcEuler start")
                    s_historyTargets[0].correctTrajectory_and_calcEuler();
                    DEBUG("correctTrajectory_and_calcEuler end")
                    rYaw = s_historyTargets[0].rYaw;
                    rPitch = s_historyTargets[0].rPitch;

                    /* 7.射击策略 */
                    if (s_historyTargets.size() >= 3 &&
                        cv::abs(s_historyTargets[0].ptsInShoot.x) < 70.0 &&
                        cv::abs(s_historyTargets[0].ptsInShoot.y) < 60.0 &&
                        cv::abs(s_historyTargets[1].ptsInShoot.x) < 120.0 && cv::abs(s_historyTargets[1].ptsInShoot.y) < 90.0)
                        statusA = SEND_STATUS_AUTO_SHOOT;   //射击

                    m_is.addText(cv::format("ptsInGimbal: %2.3f %2.3f %2.3f",
                                            s_historyTargets[0].ptsInGimbal.x / 1000.0,
                                            s_historyTargets[0].ptsInGimbal.y / 1000.0,
                                            s_historyTargets[0].ptsInGimbal.z / 1000.0));
                    m_is.addText(cv::format("rPitch %.3f", rPitch));
                    m_is.addText(cv::format("rYaw   %.3f", rYaw));
                    m_is.addText(cv::format("gYaw   %.3f", gYaw));
                    m_is.addText(cv::format("rYaw + gYaw   %.3f", rYaw - gYaw));
                }
                /* 8.通过PID对yaw进行修正（参数未修改） */
                float newYaw = rYaw;
                if (cv::abs(rYaw) < 5)
                    newYaw = m_pid.calc(rYaw, timeStamp);
                else
                    m_pid.clear();
                m_is.addText(cv::format("newYaw %3.3f", newYaw));
                m_is.addText(cv::format("delta yaw %3.3f", newYaw - rYaw));

                newYaw = cv::abs(newYaw) < 0.3 ? rYaw : newYaw;

                /* 9.发给电控 */
                m_communicator.send(newYaw, rPitch, statusA, SEND_STATUS_WM_PLACEHOLDER);
                PRINT_INFO("[attack] send = %ld", timeStamp);
            }
            if (preLock.owns_lock())
                preLock.unlock();
            return true;
        }
    };
} 
#endif
