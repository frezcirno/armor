#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#include "google/protobuf/wrappers.pb.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#pragma GCC diagnostic pop

#include "debug.h"

// TODO: 把这些静态常量移到配置文件中
constexpr int fixedSize = 32;
/*模型路径*/
const std::string model_path = "../Model/happyModel.pb";
/*输入输出节点详见ipynb的summary*/
const std::string input_name = "input_1:0";
const std::string output_name = "y/Sigmoid:0";

class TfClassifier {
  private:
    tensorflow::Session *session;  // 分类器
    static int s_cropNameCounter;  // 存文件时候使用

    /**
     * @param image 图片
     * @param t tensor
     * 将图片从mat转化为tensor
     */
    static void mat2Tensor(const cv::Mat &image, tensorflow::Tensor &t) {
        float *tensor_data_ptr = t.flat<float>().data();
        cv::Mat fake_mat(image.rows, image.cols, CV_32FC(image.channels()), tensor_data_ptr);
        image.convertTo(fake_mat, CV_32FC(image.channels()));
    }

    /**
     * @param mat 图片
     * @param thre_proportion 比例阈值 0.1
     * 得到二值化阈值
     * @return i 二值化阈值
     */
    static int getThreshold(const cv::Mat &mat, double thre_proportion = 0.1) {
        /* 计算总像素数目 */
        uint32_t iter_rows = mat.rows;
        uint32_t iter_cols = mat.cols;
        auto sum_pixel = iter_rows * iter_cols;
        /* 判断是否连续*/
        if (mat.isContinuous()) {
            iter_cols = sum_pixel;
            iter_rows = 1;
        }
        /* 新建数组置零 */
        int histogram[256];
        memset(histogram, 0, sizeof(histogram));
        /* 像素排序 */
        for (uint32_t i = 0; i < iter_rows; ++i) {
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
     * @param img 图片
     * @param result
     * 进行图片的预处理和高光补偿
     * @return true/false
     */
    static bool loadAndPre(cv::Mat img, cv::Mat &result) {
        if (img.rows == 0 || img.cols == 0)
            return false;
        /* 调整大小 同比缩放至fixedsize*fixedsize以内 */
        if (img.cols < img.rows) {
            cv::resize(img, img, {ceil(float(fixedSize) * img.cols / img.rows), fixedSize});
        } else {
            cv::resize(img, img, {fixedSize, ceil(float(fixedSize) * img.rows / img.cols)});
        }
        /* 剪去边上多余部分 */
        int cutRatio1 = 0.15 * img.cols;
        int cutRatio2 = 0.05 * img.rows;
        cv::Mat blank = cv::Mat(cv::Size(fixedSize, fixedSize), img.type(), cv::Scalar(0));                            //新建空白
        cv::Mat mask = img(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2));        //建立腌摸
        cv::Mat imageROI = blank(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2));  //建立需要覆盖区域的ROI
        mask.copyTo(imageROI, mask);
        int thre = getThreshold(blank);  //均值获取阈值
        result = blank.clone();
        /* 使用二值化阈值补高光 */
        for (int i = 0; i < result.rows; i++) {
            for (int j = 0; j < result.cols; j++) {
                if ((int)result.at<u_char>(i, j) > thre)
                    result.at<u_char>(i, j) = 200;
            }
        }
        return true;
    }

    /**
     * 初始化一个session
     */
    void initTFSession() {
        using namespace tensorflow;
        Status s;

        s = NewSession(SessionOptions(), &session);
        if (!s.ok()) {
            PRINT_ERROR("[TensorFlow] %s\n", s.ToString().c_str());
            return;
        } else {
            PRINT_INFO("[TensorFlow] Create session successfully\n");
        }

        /* 从pb文件中读取模型 */
        GraphDef graph_def;
        s = ReadBinaryProto(Env::Default(), model_path, &graph_def);  //读取Graph, 如果是文本形式的pb,使用ReadTextProto
        if (!s.ok()) {
            PRINT_ERROR("[TensorFlow] %s\n", s.ToString().c_str());
            return;
        } else {
            PRINT_INFO("[TensorFlow] Load graph protobuf successfully\n");
        }

        /* 将模型设置到创建的Session里 */
        s = session->Create(graph_def);
        if (!s.ok()) {
            PRINT_ERROR("%s\n", s.ToString().c_str());
            return;
        } else {
            PRINT_INFO("[TensorFlow] Add graph to session successfully\n");
        }
    }

  public:
    TfClassifier() {
        initTFSession();
    }

    ~TfClassifier() {
        session->Close();
    }

    /**
     * 基于tensorflow的分类器
     * @param isSave 是否保存样本图片
     * @change targets 经过分类器的装甲板
     */
    void m_classify_single_tensor(const cv::Mat &m_bgr_raw, const std::vector<Target> &preTargets, std::vector<Target> &targets, ImageShowClient &is) {
        using namespace tensorflow;

        if (preTargets.empty())
            return;
        auto input = Tensor(DT_FLOAT, TensorShape({1, fixedSize, fixedSize, 1}));

        for (auto &_tar : preTargets) {
            auto pixelPts2f_Ex_array = _tar.pixelPts2f_Ex.toArray();
            cv::Rect tmp = cv::boundingRect(pixelPts2f_Ex_array);
            cv::Mat tmp2 = m_bgr_raw(tmp).clone();
            /* 将图片变成目标大小 */
            cv::Mat transMat = cv::getPerspectiveTransform(pixelPts2f_Ex_array, pixelPts2f_Ex_array);
            cv::Mat _crop;
            /* 投影变换 */
            cv::warpPerspective(tmp2, _crop, transMat, cv::Size(tmp2.size()));
            is.addImg("crop", _crop);
            /* 转灰度图 */
            cv::cvtColor(_crop, _crop, cv::COLOR_BGR2GRAY);
            cv::Mat image;
            if (loadAndPre(_crop, image)) {
                /* mat转换为tensor */
                mat2Tensor(image, input);
                /* 保留最终输出 */
                std::vector<Tensor> outputs;
                /* 计算最后结果 */
                TF_CHECK_OK(session->Run({std::make_pair(input_name, input)}, {output_name}, {}, &outputs));
                /* 获取输出 */
                auto output_c = outputs[0].scalar<float>();
                float result = output_c();
                /* 判断正负样本 */
                if (0.3 <= result) {
                    targets.emplace_back(_tar);
                }
                /* 储存图 */
                if (false) {
                    cv::imwrite(cv::format("../data/raw/%d_%d.png", s_cropNameCounter++, 0 <= result), _crop);
                }
            } else {
                continue;
            }
        }
        is.addClassifiedTargets("After Classify", targets);
        std::cout << "Targets: " << targets.size() << std::endl;
    }
};

int TfClassifier::s_cropNameCounter = 0;
