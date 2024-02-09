#pragma once

#include "m3t/camera.h"
#include "m3t/clock.h"
#include "m3t/common.h"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <ranges>
#include <string>
#include <vector>

namespace m3t {

inline std::vector<steady_time_point> parse_timestamps(
    std::filesystem::path const &timestamp_file) {
  assert(std::filesystem::exists(timestamp_file) &&
         std::filesystem::is_regular_file(timestamp_file));

  std::ifstream file(timestamp_file);
  assert(file.is_open());

  auto lines = std::ranges::istream_view<std::string>(file);
  auto time_points =
      lines | std::views::transform([](std::string const &line) {
        auto millis_count = std::stoull(line);
        return steady_time_point(std::chrono::milliseconds(millis_count));
      });

  std::vector<steady_time_point> result{};
  for (auto time_point : time_points) {
    result.push_back(time_point);
  }

  assert(result.size() > 0);
  return result;
}

class VideoFileCamera : public ColorCamera {
 public:
  explicit VideoFileCamera(
      std::string const &name, std::filesystem::path path,
      std::vector<std::chrono::steady_clock::time_point> frame_times,
      Clock *clock, Intrinsics intrinsics,
      std::vector<double> distortion_coefficients);

  // Setup method
  bool SetUp() override;

  // Main methods
  bool UpdateImage(bool synchronized) override;

  bool readFrame(size_t frame_index);

  ~VideoFileCamera() override;

 private:
  std::size_t findClosestFrame(steady_time_point time);
  void createCapture();

  std::filesystem::path path_;
  std::vector<steady_time_point> frame_times_;
  Clock *clock_;

  cv::Mat camera_matrix_{};
  std::vector<double> distortion_coefficients_;
  cv::Mat undistort_rectify_map_x_;
  cv::Mat undistort_rectify_map_y_;

  cv::VideoCapture capture_;
  std::optional<std::size_t> current_frame_index_{};
};

inline std::vector<steady_time_point> getTimestampsFromConfig(std::filesystem::path const &config_path) {
  cv::FileStorage fs{};
  OpenYamlFileStorage(config_path, &fs);
  std::filesystem::path timestamp_path;
  ReadRequiredValueFromYaml(fs, "timestamp_path", &timestamp_path);
  return parse_timestamps(timestamp_path);
}

inline VideoFileCamera createVideoFileCameraFromConfig(
    std::filesystem::path const &config_path, Clock *clock) {
  std::string name;
  std::filesystem::path video_path;
  std::filesystem::path timestamp_path;
  Intrinsics intrinsics{};
  std::vector<double> distortion_coefficients;
  auto camera2world_pose = m3t::Transform3fA::Identity();

  cv::FileStorage fs{};
  OpenYamlFileStorage(config_path, &fs);

  ReadRequiredValueFromYaml(fs, "name", &name);
  ReadRequiredValueFromYaml(fs, "video_path", &video_path);
  ReadRequiredValueFromYaml(fs, "timestamp_path", &timestamp_path);
  ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics);
  ReadRequiredValueFromYaml(fs, "distortion_coefficients",
                            &distortion_coefficients);
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose);

  auto camera =
      VideoFileCamera{name,  video_path, parse_timestamps(timestamp_path),
                      clock, intrinsics, distortion_coefficients};
  camera.set_camera2world_pose(camera2world_pose);

  return camera;
}

}  // namespace m3t
