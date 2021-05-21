#include <cstdlib>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

int main(int argc, char **argv) {
  cv::theRNG().state = time(nullptr);
  cv::Mat mat(100, 100, CV_8UC3);
  cv::randu(mat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

  cv::imwrite("test.jpg", mat);
}
