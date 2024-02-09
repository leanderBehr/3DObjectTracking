//
// Created by leander.behr on 04.12.2023.
//

#include "m3t/cameras/video_file_camera.h"
#include <opencv2/photo/cuda.hpp>


namespace m3t {
    VideoFileCamera::VideoFileCamera(
            std::string const &name,
            std::filesystem::path path,
            std::vector<std::chrono::steady_clock::time_point> frame_times,
            Clock *clock,
            Intrinsics intrinsics,
            std::vector<double> distortion_coefficients) :
            ColorCamera(name),
            path_(std::move(path)),
            frame_times_(std::move(frame_times)),
            clock_(clock),
            distortion_coefficients_(std::move(distortion_coefficients)) {
        intrinsics_ = intrinsics;
        cv::Mat_<float> intrinsics_mat(3, 3);
        intrinsics_mat <<
                intrinsics.fu, 0, intrinsics.ppu,
                0, intrinsics.fv, intrinsics.ppv,
                0, 0, 1;

        camera_matrix_ = intrinsics_mat;
        cv::initUndistortRectifyMap(
            camera_matrix_, distortion_coefficients_, cv::Mat(), camera_matrix_,
            cv::Size(intrinsics_.width, intrinsics_.height), CV_32FC1,
            undistort_rectify_map_x_, undistort_rectify_map_y_);
    }

    bool VideoFileCamera::SetUp() {
        createCapture();
        bool success = capture_.isOpened();
        set_up_ = success;
        return success;
    }

    bool VideoFileCamera::UpdateImage(bool synchronized) {
        if (!set_up_)
            return false;

        auto frame_index = findClosestFrame(clock_->current_time());

        if (!readFrame(frame_index)) return false;

        SaveImageIfDesired();
        return true;
    }

    bool VideoFileCamera::readFrame(size_t frame_index) {
        // repeat frame
        if (current_frame_index_ && *current_frame_index_ == frame_index) return true;

        // jump back
        if (capture_.get(cv::CAP_PROP_POS_FRAMES) > frame_index) {
            capture_.set(cv::CAP_PROP_POS_FRAMES,
                         static_cast<double>(frame_index));
        }

        // move forward
        while (capture_.get(cv::CAP_PROP_POS_FRAMES) < frame_index) {
            cv::Mat discard;
            capture_.read(discard);
        }

        // read frame
        current_frame_index_ = capture_.get(cv::CAP_PROP_POS_FRAMES);
        cv::Mat current_frame;
        capture_.read(current_frame);

        if (current_frame.empty()) return false;

        // denoise
        //cv::fastNlMeansDenoisingColored(current_frame, current_frame, 10);

        // undistort
        cv::remap(current_frame, image_, undistort_rectify_map_x_,
                  undistort_rectify_map_y_, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 255, 0));
        //cv::undistort(current_frame, image_, camera_matrix_, distortion_coefficients_, camera_matrix_);
        
        return true;
    }

    VideoFileCamera::~VideoFileCamera() {
        capture_.release();
    }

    std::size_t VideoFileCamera::findClosestFrame(steady_time_point time) {
        auto first_greater_equal = std::ranges::lower_bound(frame_times_, time);
        auto first_greater_equal_idx = std::distance(frame_times_.begin(), first_greater_equal);
        if(first_greater_equal_idx > 0)
            first_greater_equal_idx -= 1;
        return first_greater_equal_idx;
    }

    void VideoFileCamera::createCapture() {
        capture_ = cv::VideoCapture(path_.string());
        capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        capture_.set(cv::CAP_PROP_BUFFERSIZE, std::pow(2, 20) * 200);
    }
} // m3t