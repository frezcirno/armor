//
// Created by sp on 19-6-16.
// 摄像头相关调试
//
#include <omp.h>
#include "capture.hpp"

static double computeReprojectionErrors(
        const std::vector<std::vector<cv::Point3f> > &objectPoints,
        const std::vector<std::vector<cv::Point2f> > &imagePoints,
        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
        std::vector<float> &perViewErrors) {
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int) objectPoints.size(); i++) {
        projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
        int n = (int) objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

static void
calcChessboardCorners(const cv::Size &boardSize, float squareSize, std::vector<cv::Point3f> &corners) {
    corners.resize(0);
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            corners.emplace_back(cv::Point3f(float(j * squareSize),
                                             float(i * squareSize), 0));

}

static bool runCalibration(const std::vector<std::vector<cv::Point2f>> &imagePoints,
                           cv::Size &imageSize, cv::Size &boardSize,
                           float squareSize, float aspectRatio,
                           int flags, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                           std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs,
                           std::vector<float> &reprojErrs, double &totalAvgErr) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = aspectRatio;

    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

static void saveCameraParams(const cv::String &filename,
                             cv::Size imageSize, cv::Size boardSize,
                             float squareSize, float aspectRatio, int flags,
                             const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                             const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                             const std::vector<float> &reprojErrs,
                             const std::vector<std::vector<cv::Point2f>> &imagePoints,
                             double totalAvgErr) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nframes" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        cv::Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int) rvecs.size(); i++) {
            cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
            cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if (!imagePoints.empty()) {
        cv::Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int) imagePoints.size(); i++) {
            cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

static bool runAndSave(const cv::String &outputFilename,
                       const std::vector<std::vector<cv::Point2f>> &imagePoints,
                       cv::Size &imageSize, cv::Size &boardSize, float squareSize,
                       float aspectRatio, int flags, cv::Mat &cameraMatrix,
                       cv::Mat &distCoeffs, bool writeExtrinsics, bool writePoints) {
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if (ok)
        saveCameraParams(outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : std::vector<cv::Mat>(),
                         writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
                         writeExtrinsics ? reprojErrs : std::vector<float>(),
                         writePoints ? imagePoints : std::vector<std::vector<cv::Point2f> >(),
                         totalAvgErr);
    return ok;
}

int main() {
    std::cout << "Using OpenCV " << CV_VERSION << std::endl;

    cv::Size boardSize = cv::Size(7, 8);
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f> > imagePoints;
    cv::Size imageSize = cv::Size(1280, 1024);
    float squareSize = 25.0;  // mm
    int flags = 0;

    armor::MindVision capture;
    capture.init();
    capture.play();

    cv::Mat frame, view;
    capture.initFrameMat(frame);
    int64 timeStamp;
    bool enableRealTimeFind = false;
    int fileNameCounter = 0;
    while (capture.isOpened()) {
        if (capture.wait_and_get(frame, timeStamp, []() {})) {
            frame.copyTo(view);
            if (enableRealTimeFind) {
                bool found = findChessboardCorners(view, boardSize, corners,
                                                   cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                   cv::CALIB_CB_NORMALIZE_IMAGE);
                drawChessboardCorners(view, boardSize, cv::Mat(corners), found);
            }
            cv::putText(view, cv::format("RealTimeFind <f> %d", enableRealTimeFind), cv::Point(12, 12),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(120, 255, 120));
            cv::putText(view, cv::format("fileNameCounter <s> %d", fileNameCounter), cv::Point(12, 36),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(120, 255, (fileNameCounter * 20) % 255));
            cv::putText(view, cv::format("press <q> quit to run calibration"), cv::Point(12, 48),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(120, 255, (fileNameCounter * 20) % 255));
            cv::imshow("main", view);
        }
        int k = cv::waitKey(1);
        if (k == 'q') break;
        else if (k == 'f') enableRealTimeFind = !enableRealTimeFind;
        else if (k == 's') cv::imwrite(cv::format("../data/CalibrationImages/%d.png", fileNameCounter++), frame);
        else if (k == ' ') {
            while (cv::waitKey(0) != ' ') {}
        }
    }

    std::vector<cv::String> results;
    cv::glob("../data/CalibrationImages/", results);
#pragma omp parallel for
    for (int i = 0; i < results.size(); i++) {
        cv::Mat img;
        printf("%s\n", results[i].c_str());
        img = cv::imread(results[i]);
        bool found = findChessboardCorners(img, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH |
                                                                    cv::CALIB_CB_FAST_CHECK |
                                                                    cv::CALIB_CB_NORMALIZE_IMAGE);
        // improve the found corners' coordinate accuracy
        if (found) {
            cv::Mat viewGray;
            cv::cvtColor(img, viewGray, cv::COLOR_BGR2GRAY);
            cornerSubPix(viewGray, corners, cv::Size(11, 11),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(view, boardSize, cv::Mat(corners), found);
#pragma omp critical
            imagePoints.emplace_back(corners);
        }
    }

    if (imagePoints.size() > 10) {
        cv::Mat cameraMatrix, distCoeffs;
        runAndSave("../data/camera.xml", imagePoints, imageSize,
                   boardSize, squareSize, 1,
                   flags, cameraMatrix, distCoeffs,
                   true, true);
        cv::Mat rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);
        while (capture.isOpened()) {
            if (capture.wait_and_get(frame, timeStamp, []() {})) {
                cv::Mat raw;
                frame.copyTo(raw);
                remap(frame, rview, map1, map2, cv::INTER_LINEAR);
                cv::imshow("raw", raw);
                cv::imshow("main", rview);
            }
            int k = cv::waitKey(1);
            if (k == 'q') break;
            else if (k == ' ') {
                while (cv::waitKey(0) != ' ') {}
            }
        }

    } else {
        printf("imagePoints not enough\n");
    }

    return 0;
}
