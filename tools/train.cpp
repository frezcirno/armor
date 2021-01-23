//
// Created by sp on 19-6-22.
// 训练分类器
//


#include <dirent.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "attack.hpp"

/**
 * 文件夹遍历, 将自动以文件夹名作为label名
 * train  # 训练文件夹, 传入此路径
 * ├── 0  # 样本0文件夹, label = atoi("0")
 * └── 1  # 样本1文件夹, label = atoi("1")
 * @param mainPath
 * @param fileNames [[label A, [0.png, ...]], [label B, [..., ...]]]
 */
void getAllFiles(const cv::String &mainPath, std::vector<std::pair<int, std::vector<cv::String>>> &fileNames) {
    const char *dir_name = mainPath.c_str();
    // check if dir_name is a valid dir
    struct stat s;
    lstat(dir_name, &s);
    if (!S_ISDIR(s.st_mode)) {
        std::cout << "dir_name is not a valid directory !" << std::endl;
        return;
    }

    DIR *dir;
    dir = opendir(dir_name);
    if (nullptr == dir) {
        std::cout << "Can not open dir " << dir_name << std::endl;
        return;
    }

    /* read all the files in the dir ~ */
    struct dirent *filename;  // return value for readdir()
    while ((filename = readdir(dir)) != nullptr) {
        if (strcmp(filename->d_name, ".") == 0 || strcmp(filename->d_name, "..") == 0)
            continue;
        cv::String _labelDir = mainPath + filename->d_name;
        std::pair<int, std::vector<cv::String>> _labelFileNames;
        _labelFileNames.first = strtol(filename->d_name, nullptr, 10);
        std::vector<cv::String> _imgNames;
        cv::glob(_labelDir, _imgNames);
        _labelFileNames.second = _imgNames;
        fileNames.emplace_back(_labelFileNames);
    }
    closedir(dir);

    std::cout << "类别数量: " << fileNames.size() << std::endl;
}

void getData(const cv::String &mainPath, int trainNumber,
             cv::Mat &trainData, cv::Mat &trainLabels,
             cv::Mat &testData, cv::Mat &testLabels) {
    std::vector<std::pair<int, std::vector<cv::String>>> fileNames;
    getAllFiles(mainPath, fileNames);

    armor::Hog hog;
    std::vector<cv::Mat> trainGradients;
    std::vector<cv::Mat> testGradients;
    for (auto &_samples:fileNames) {
        for (int i = 0; i < _samples.second.size(); ++i) {
            cv::Mat crop = cv::imread(_samples.second[i], cv::IMREAD_GRAYSCALE);
            cv::Mat _gradient;
            hog.computeHOG_Single(crop, _gradient);
            if (i < trainNumber) {
                trainGradients.emplace_back(_gradient);
                trainLabels.push_back<int>(_samples.first);
            } else {
                testGradients.emplace_back(_gradient);
                testLabels.push_back<int>(_samples.first);
            }
        }
    }
    hog.convert_to_ml(trainGradients, trainData);
    hog.convert_to_ml(testGradients, testData);
}

void trainRandomForest(const cv::String &mainPath, const cv::String &outputPath) {
    cv::Mat train_data, test_data;
    cv::Mat test_labels(cv::Size(0, 0), CV_32S);
    cv::Mat train_labels(cv::Size(0, 0), CV_32S);
    getData(mainPath, 130, train_data, train_labels, test_data, test_labels);
    cv::Ptr<cv::ml::RTrees> rtrees = cv::ml::RTrees::create();
    rtrees->setMaxDepth(20);
    rtrees->setMinSampleCount(4);
    rtrees->setRegressionAccuracy(0.f);
    rtrees->setUseSurrogates(false);
    rtrees->setMaxCategories(16);
    rtrees->setPriors(cv::Mat());
    rtrees->setCalculateVarImportance(false);
    rtrees->setActiveVarCount(0);
    rtrees->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 2000, 0));
//    rtrees->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 2500, 0));
    cv::Ptr<cv::ml::TrainData> tData = cv::ml::TrainData::create(train_data, cv::ml::ROW_SAMPLE, train_labels);

    rtrees->train(tData);
    rtrees->save(outputPath);

    std::vector<float> train_predict;
    rtrees->predict(train_data, train_predict);
    printf("rf saved in '%s'\n", outputPath.c_str());
    float ratio = 0.0;
    for (int i = 0; i < train_predict.size(); i++) {
        if (train_labels.at<int>(i) == int(train_predict[i])) {
            ratio += 1;
        }
    }
    printf("rf 训练集: %.5f \n", ratio / train_predict.size());

    std::vector<float> test_predict;
    rtrees->predict(test_data, test_predict);
    ratio = 0.0;
    for (int i = 0; i < test_predict.size(); i++) {
        if (test_labels.at<int>(i) == int(test_predict[i])) {
            ratio += 1;
        }
    }
    printf("rf 测试集: %.5f \n", ratio / test_predict.size());
}


int main() {
    std::cout << "Using OpenCV " << CV_VERSION << std::endl;

    trainRandomForest("../data/samples/", "../data/rf.xml");
    return 0;
}

