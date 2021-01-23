// by yaw 2019/03/145

#ifndef WINDMILL_HPP
#define WINDMILL_HPP


#include "base.hpp"
#include "imageshow.hpp"
#include "Target.hpp"
#include "Tool.hpp"
#include "opencv2/opencv.hpp"
//#include "../tn/tn.hpp"
#include <algorithm>

#define WIDTH 260
#define HEIGHT 200
#define RADIUS 700 // Wait to change to 740
#define ANGULAR_VELOCITY 60 / 180.0 * M_PI
#define RECT_SIZE 150
#define EPS 1e-2


#include <stdlib.h>

namespace wm {
    const cv::String PATH = "../data/windmill/";
    const cv::String FILE_TYPE = ".jpg";
    static int gNameCount = 0;
    typedef enum {
        TRACK, JUMP
    } State;

    class Windmill {
    public:
        bool run(const cv::Mat &frame, float &pitch, float &yaw,
                 double time);

        static Windmill *GetInstance(const cv::Mat &cam, const cv::Mat &dist,
                                     const cv::Mat &TvCtoL, const double delay, const cv::String &modelName,
                                     armor::ImageShowClient *is, const double maxPitchError, const double maxYawError) {
            static Windmill instance(cam, dist, TvCtoL, delay, modelName, is, maxPitchError, maxYawError);
            return &instance;
        }

        std::string mode;
        State state = JUMP;
        int stateCount = 0;
        Target findedTarget;
        Target hitTarget;
        cv::Point3f center;
        cv::Point2f center2D;
        double timeStamp = 0.0;
        double delay;
        cv::Mat camMatrix;
        cv::Mat distCoeffs;
        cv::Mat TvCtoL;
        float radius = RADIUS;
        float v = ANGULAR_VELOCITY;
        float gPitch = 0.0;
        float gYaw = 0.0;

        int sampleNumber = 0;

        double spinAngle = 0.0;//角度
        double maxPitchError = 0.0;
        double maxYawError = 0.0;

        int thre;
        bool close;
        cv::Rect rect1;
        cv::Rect rect2;
        bool isSmallROI = false;

    private:
        explicit Windmill(const cv::Mat &cam, const cv::Mat &dist, const cv::Mat &TvCtoL,
                          const double delay, const cv::String &modelName, armor::ImageShowClient *is,
                          const double maxPitchError, const double maxYawError)
                : delay(delay), camMatrix(cam), distCoeffs(dist), TvCtoL(TvCtoL), is(is), maxPitchError(maxPitchError),
                  maxYawError(maxYawError) {}


        armor::ImageShowClient *is;
    };


    bool Windmill::run(const cv::Mat &frame, float &pitch, float &yaw,
                       double time) {


        STATE(INFO, "Windmill Running", 1)
        return false;
    }

} // namespace wm
#endif
