//
// Created by leander.behr on 10.11.2023.
//

#ifndef M3T_OPENCV_CAMERA_H
#define M3T_OPENCV_CAMERA_H

#include <m3t/camera.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace m3t {

    class OpenCVCamera : public ColorCamera {
    public:
        explicit OpenCVCamera(const std::string &name, Intrinsics intrinsics,
                              std::vector<double> distortion_coefficients);

        // Setup method
        bool SetUp() override;

        // Main methods
        bool UpdateImage(bool synchronized) override;

        ~OpenCVCamera() override;

    private:
        cv::VideoCapture capture_;
        cv::Mat camera_matrix_{};
        std::vector<double> distortion_coefficients_;
    };
} // m3t

#endif //M3T_OPENCV_CAMERA_H
