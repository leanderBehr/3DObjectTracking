//
// Created by leander.behr on 10.11.2023.
//

#include "m3t/cameras/opencv_camera.h"


namespace m3t {
    OpenCVCamera::OpenCVCamera(const std::string &name,
                               Intrinsics intrinsics,
                               std::vector<double> distortion_coefficients) :
            ColorCamera(name),
            distortion_coefficients_(std::move(distortion_coefficients)) {
        intrinsics_ = intrinsics;
        cv::Mat_<float> intrinsics_mat(3, 3);
        intrinsics_mat <<
                intrinsics.fu, 0, intrinsics.ppu,
                0, intrinsics.fv, intrinsics.ppv,
                0, 0, 1;

        camera_matrix_ = intrinsics_mat;
    }

    bool OpenCVCamera::SetUp() {
        capture_ = cv::VideoCapture(0);
        bool success = capture_.isOpened();
        set_up_ = success;
        return success;
    }

    bool OpenCVCamera::UpdateImage(bool synchronized) {
        if (!set_up_)
            return false;

        cv::Mat new_frame;
        capture_ >> new_frame;
        if (new_frame.empty())
            return false;

        //if (image_.empty())
        //    image_ = cv::Mat(new_frame.rows, new_frame.cols, CV_64F);

        //cv::undistort(new_frame, image_, camera_matrix_, distortion_coefficients_);
        cv::undistort(new_frame, image_, camera_matrix_, distortion_coefficients_, camera_matrix_);
        //image_ = new_frame;
        SaveImageIfDesired();
        return true;
    }

    OpenCVCamera::~OpenCVCamera() {
        capture_.release();
    }
} // m3t