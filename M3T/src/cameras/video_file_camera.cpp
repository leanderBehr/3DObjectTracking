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
    std::vector<double> distortion_coefficients,
    bool flip) : ColorCamera(name),
                 path_(std::move(path)),
                 frame_times_(std::move(frame_times)),
                 clock_(clock),
                 distortion_coefficients_(std::move(distortion_coefficients)),
                 flip_(flip) {
  //intrinsics_ = intrinsics;
  cv::Mat_<float> intrinsics_mat(3, 3);
  intrinsics_mat << intrinsics.fu, 0, intrinsics.ppu,
      0, intrinsics.fv, intrinsics.ppv,
      0, 0, 1;

  cv::Mat camera_matrix = intrinsics_mat;
  
  cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients_, cv::Size(intrinsics_.width, intrinsics_.height), 0);
  
  intrinsics_ = Intrinsics{
      .fu = new_camera_matrix.at<float>(0, 0),
      .fv = new_camera_matrix.at<float>(1, 1),
      .ppu = new_camera_matrix.at<float>(0, 2),
      .ppv = new_camera_matrix.at<float>(1, 2),
      .width = intrinsics.width,
      .height = intrinsics.height
  };

  cv::initUndistortRectifyMap(
      camera_matrix, distortion_coefficients_, cv::Mat(), new_camera_matrix,
      cv::Size(intrinsics_.width, intrinsics_.height), CV_32FC1,
      undistort_rectify_map_x_, undistort_rectify_map_y_);



  if (flip_)
    intrinsics_.ppv = intrinsics_.height - intrinsics_.ppv;
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
    capture_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frame_index));
  }

  // jump forward if more than 50 frames behind
  if (capture_.get(cv::CAP_PROP_POS_FRAMES) + 50 < frame_index) {
    capture_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frame_index));
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

  // undistort
  cv::remap(current_frame, image_, undistort_rectify_map_x_,
            undistort_rectify_map_y_, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 255, 0));

  //image_ = current_frame;

  if (flip_)
    cv::flip(image_, image_, 0);

  return true;
}

VideoFileCamera::~VideoFileCamera() {
  capture_.release();
}

std::size_t VideoFileCamera::findClosestFrame(steady_time_point time) {
  auto first_greater_equal = std::ranges::lower_bound(frame_times_, time);
  auto first_greater_equal_idx = std::distance(frame_times_.begin(), first_greater_equal);
  if (first_greater_equal_idx > 0)
    first_greater_equal_idx -= 1;
  return first_greater_equal_idx;
}

void VideoFileCamera::createCapture() {
  capture_ = cv::VideoCapture(path_.string());
  capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  capture_.set(cv::CAP_PROP_BUFFERSIZE, std::pow(2, 20) * 200);
}
}  // namespace m3t