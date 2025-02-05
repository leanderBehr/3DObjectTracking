// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/manual_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>
#include "m3t/cameras/opencv_camera.h"
#include "m3t/cameras/video_file_camera.h"
#include "m3t/lambda_subscriber.h"
#include "m3t/pose_logger.h"

#include "Settings.h"

#include <Eigen/Geometry>

#include <filesystem>
#include <memory>
#include <numbers>
#include <ranges>
#include <string>
#include <vector>

struct Cameras {
  std::unique_ptr<m3t::Clock> clock;
  std::vector<std::unique_ptr<m3t::ColorCamera>> cameras{};
};

auto range_to_vector(std::ranges::range auto &&range) {
  return std::vector(range.begin(), range.end());
}

Cameras makeCameras(std::filesystem::path const &base_directory,
                    std::vector<std::string> camera_names) {
  auto timestamp_vectors_view =
      camera_names | std::views::transform([&](auto camera_name) {
        return m3t::getTimestampsFromConfig(base_directory / std::format("{}.yaml", camera_name));
      });
  std::vector<std::vector<m3t::steady_time_point>> timestamp_vectors = range_to_vector(timestamp_vectors_view);
  auto all_timestamps = std::views::join(timestamp_vectors);
  auto min_max = std::ranges::minmax_element(all_timestamps);

  auto cameras = Cameras{
      .clock = std::make_unique<m3t::Clock>(*min_max.min, *min_max.max)};

  auto color_camera_ptr_from_path =
      [&](auto camera_name) -> std::unique_ptr<m3t::ColorCamera> {
    return std::make_unique<m3t::VideoFileCamera>(createVideoFileCameraFromConfig(
        base_directory / std::format("{}.yaml", camera_name),
        cameras.clock.get()));
  };

  auto cameras_view = camera_names | std::views::transform(color_camera_ptr_from_path);

  cameras.cameras = range_to_vector(cameras_view);
  return cameras;
}

Cameras makeWebcam() {
  auto camera = std::make_unique<m3t::OpenCVCamera>(
      "opencv_camera",
      m3t::Intrinsics{.fu = 619.65f,
                      .fv = 623.52f,
                      .ppu = 315.73f,
                      .ppv = 231.98f,
                      .width = 640,
                      .height = 480},
      std::vector{-0.09060528, 0.86039354, 0.00231482, 0.00488739,
                  -2.11455125});

  std::vector<std::unique_ptr<m3t::ColorCamera>> camera_vec{};
  camera_vec.push_back(std::move(camera));

  auto clock = std::make_unique<m3t::Clock>(std::chrono::steady_clock::now(),
                                            m3t::steady_time_point::max());

  return Cameras{.clock = std::move(clock), .cameras = std::move(camera_vec)};
}

auto good_eval_pointer_settings = Settings{
    .modality_settings = {
        .n_histogram_bins = 64,

        // 20s for big frame to frame jumps and recovery from losing tracking close to the image border
        .scales = std::vector{20, 20, 12, 12, 8, 8, 4, 4},
        .standard_deviations = std::vector<float>{20, 20, 15, 15, 7, 7, 4, 4},

        .max_considered_line_length = 200,
        .unconsidered_line_length = 5,

        .function_slope = 2,
        .function_amplitude = 0.3,

        .n_global_iterations = 6,

        .n_lines_max = 400,
        .min_continuous_distance = 1,
    },
    .optimizer_settings = {
        .tikhonov_parameter_translation = 5 * 1e7,
        .tikhonov_parameter_rotation = 1 * 1e5,
    },
    .tracker_settings = {
        .n_update_iterations = 6,
    },
};

auto examples_settings = Settings{
    .modality_settings = {
        .n_histogram_bins = 64,

        .scales = std::vector{16, 16, 8, 8, 4, 4, 2, 2},
        .standard_deviations = std::vector<float>{16, 16, 8, 8, 4, 4, 2, 2},

        .max_considered_line_length = 40,
        .unconsidered_line_length = 0.5,

        .function_slope = 0.5,      // (0, infity), 0 => small local uncertainty, infity => large local uncertainty
        .function_amplitude = 0.4,  // (0, 0.5], 0 => large global noise, 0.5 => small global noise

        .n_global_iterations = 3,

        .n_lines_max = 400,
        .min_continuous_distance = 1,
    },
    .optimizer_settings = {
        .tikhonov_parameter_translation = 5 * 1e7,
        .tikhonov_parameter_rotation = 1 * 1e5,
    },
    .tracker_settings = {
        .n_update_iterations = 6,
    },
};

auto eval_settings = Settings{
    .modality_settings = {
        .n_histogram_bins = 64,

        .scales = std::vector{8, 8, 4, 4, 2, 2},
        .standard_deviations = std::vector<float>{8, 8, 4, 4, 2, 2},

        .max_considered_line_length = 100,
        .unconsidered_line_length = 0,

        .function_slope = 0.4, // (0, infity), 0 => small local uncertainty, infity => large local uncertainty
        .function_amplitude = 0.5, // (0, 0.5], 0 => large global noise, 0.5 => small global noise

        .n_global_iterations = 3,

        .n_lines_max = 400,
        .min_continuous_distance = 1,
    },
    .optimizer_settings = {
        .tikhonov_parameter_translation = 5 * 1e6,
        .tikhonov_parameter_rotation = 1 * 1e5,
    },
    .tracker_settings = {
        .n_update_iterations = 6,
    },
};

auto settings = eval_settings;

void configure_region_modality(m3t::RegionModality &modality) {
  auto s = settings.modality_settings;
  if (settings.modality_settings.scales || settings.modality_settings.standard_deviations) {
    bool valid = settings.modality_settings.scales && settings.modality_settings.standard_deviations &&
                 settings.modality_settings.scales->size() ==
                     settings.modality_settings.standard_deviations->size();
    if (!valid) {
      std::cerr << "Different number of scales and standard deviations\n";
      throw std::logic_error("Different number of scales and standard deviations");
    }
  }

  if (s.n_histogram_bins) modality.set_n_histogram_bins(*s.n_histogram_bins);
  if (s.scales) modality.set_scales(*s.scales);
  if (s.standard_deviations) modality.set_standard_deviations(*s.standard_deviations);
  if (s.max_considered_line_length) modality.set_max_considered_line_length(*s.max_considered_line_length);
  if (s.unconsidered_line_length) modality.set_unconsidered_line_length(*s.unconsidered_line_length);
  if (s.function_slope) modality.set_function_slope(*s.function_slope);
  if (s.function_amplitude) modality.set_function_amplitude(*s.function_amplitude);
  if (s.n_global_iterations) modality.set_n_global_iterations(*s.n_global_iterations);
  if (s.n_lines_max) modality.set_n_lines_max(*s.n_lines_max);
  if (s.min_continuous_distance) modality.set_min_continuous_distance(*s.min_continuous_distance);
  // modality.set_learning_rate_f(0.4);
  // modality.set_learning_rate_b(0.4);
}

void configure_optimizer(m3t::Optimizer &optimizer) {
  auto s = settings.optimizer_settings;

  if (s.tikhonov_parameter_rotation) optimizer.set_tikhonov_parameter_rotation(*s.tikhonov_parameter_rotation);
  if (s.tikhonov_parameter_translation) optimizer.set_tikhonov_parameter_translation(*s.tikhonov_parameter_translation);
}

void configure_tracker(m3t::Tracker &tracker) {
  auto s = settings.tracker_settings;

  bool has_corr_it = settings.modality_settings.scales || settings.modality_settings.standard_deviations;

  if (has_corr_it) tracker.set_n_corr_iterations(settings.modality_settings.scales->size());
  if (s.n_update_iterations) tracker.set_n_update_iterations(*s.n_update_iterations);
}

using namespace std::literals;

constexpr std::optional<m3t::steady_time_point> reset_time = {};
// coated, cot, no trocar, no bone, combined => z-rotation
// constexpr std::optional<m3t::steady_time_point> reset_time = m3t::steady_time_point(std::chrono::nanoseconds(40200000000)) - std::chrono::milliseconds(500);
// constexpr std::optional<m3t::steady_time_point> reset_time = m3t::steady_time_point(std::chrono::nanoseconds(40200000000)) - std::chrono::milliseconds(500);

// ???
// constexpr std::optional<m3t::steady_time_point> reset_time = m3t::steady_time_point(std::chrono::nanoseconds(1705487399339000000)) - std::chrono::milliseconds(500);

// coated_hook_2 before jump
// constexpr std::optional<m3t::steady_time_point> reset_time = m3t::steady_time_point(std::chrono::nanoseconds(1705487406679000000)) - std::chrono::milliseconds(300);

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Not enough arguments: Provide directory and body_names";
    return -1;
  }
  std::vector<std::string> body_names;
  for (int i = 2; i < argc; ++i) {
    body_names.emplace_back(argv[i]);
  }

  auto body_of_interest = body_names.front();

  static constexpr bool kLogPoses = false;
  static constexpr bool kUseConstraints = false;
  constexpr bool kUseDepthViewer = false;
  constexpr bool kUseRegionModality = true;
  constexpr bool kUseTextureModality = false;
  constexpr bool kUseDepthModality = false;
  constexpr bool kMeasureOcclusions = false;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualize = false;
  constexpr bool kSaveImages = false;
  const std::filesystem::path save_directory{""};
  const std::filesystem::path base_directory{argv[1]};

  // Set up tracker and renderer geometry
  auto tracker_ptr = std::make_shared<m3t::Tracker>("tracker");
  configure_tracker(*tracker_ptr);

  auto renderer_geometry_ptr = std::make_shared<m3t::RendererGeometry>("renderer geometry");

  // Set up cameras
  auto cameras_ptr = std::make_shared<Cameras>(makeCameras(base_directory, {"cot_spine"}));
  // auto cameras_ptr = std::make_shared<Cameras>(makeWebcam());
  auto camera_ptrs_view = cameras_ptr->cameras | std::views::transform([&](auto &camera_ptr) {
                            return std::shared_ptr<m3t::ColorCamera>(cameras_ptr, camera_ptr.get());
                          });

  auto color_camera_ptrs = range_to_vector(camera_ptrs_view);
  auto color_camera_ptr = color_camera_ptrs.front();

  // Set up viewers
  for (auto color_camera_ptr : color_camera_ptrs) {
    auto color_viewer_ptr = std::make_shared<m3t::NormalColorViewer>(
        std::format("color_viewer_{}", color_camera_ptr->name()),
        color_camera_ptr, renderer_geometry_ptr);

    if constexpr (kSaveImages)
      color_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(color_viewer_ptr);
  }

  // Set up depth renderer
  auto color_depth_renderer_ptr = std::make_shared<m3t::FocusedBasicDepthRenderer>(
      "color_depth_renderer",
      renderer_geometry_ptr,
      color_camera_ptr);

  // Set up silhouette renderer
  auto color_silhouette_renderer_ptr = std::make_shared<m3t::FocusedSilhouetteRenderer>(
      "color_silhouette_renderer",
      renderer_geometry_ptr,
      color_camera_ptr);
  color_silhouette_renderer_ptr->set_z_min(0.0001f);
  color_silhouette_renderer_ptr->set_z_max(0.5f);

  for (const auto body_name : body_names) {
    // Set up body
    std::filesystem::path metafile_path{base_directory / (body_name + ".yaml")};
    auto body_ptr = std::make_shared<m3t::Body>(body_name, metafile_path);
    renderer_geometry_ptr->AddBody(body_ptr);
    color_depth_renderer_ptr->AddReferencedBody(body_ptr);
    color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);

    // Set up models
    auto region_model_ptr{std::make_shared<m3t::RegionModel>(
        body_name + "_region_model", body_ptr,
        base_directory / (body_name + "_region_model.bin"),
        0.8,  // sphere radius
        4,    // divides
        400   // points
        )};

    // Set up modalities
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    configure_region_modality(*region_modality_ptr);

    auto texture_modality_ptr = std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality",
        body_ptr,
        color_camera_ptr,
        color_silhouette_renderer_ptr);

    texture_modality_ptr->set_descriptor_type(m3t::TextureModality::DescriptorType::ORB_CUDA);

    if constexpr (kVisualize) {
      region_modality_ptr->set_visualize_lines_correspondence(true);
    }

    if constexpr (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      texture_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
    }

    // Set up link
    auto link_ptr = std::make_shared<m3t::Link>(body_name + "_link", body_ptr);
    if constexpr (kUseRegionModality)
      link_ptr->AddModality(region_modality_ptr);
    if constexpr (kUseTextureModality)
      link_ptr->AddModality(texture_modality_ptr);

    if constexpr (kUseConstraints) {
      link_ptr->set_free_directions({
          false, false, true,  // rotation
          true, true, true,    // translation
      });
    }

    // Set up optimizer
    auto optimizer_ptr = std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr);
    configure_optimizer(*optimizer_ptr);

    tracker_ptr->AddOptimizer(optimizer_ptr);

    // Set up detector
    std::filesystem::path detector_path = base_directory / (body_name + "_detector.yaml");

    auto detector_ptr = std::make_shared<m3t::StaticDetector>(
        body_name + "_detector",
        detector_path,
        optimizer_ptr
        /*, color_camera_ptr*/
    );
    tracker_ptr->AddDetector(detector_ptr);
  }

  // Video Control
  bool started = false;

  tracker_ptr->AddKeyCallback("StopVideo", [&](char key) {
    if (key == 'n') {
      started = false;
      std::cout << "Video Stopped "
                << cameras_ptr->clock->current_time().time_since_epoch().count()
                << "\n";
    }
  });

  tracker_ptr->AddKeyCallback("StartVideo", [&](char key) {
    if (key == 'm') {
      started = true;
      std::cout << "Video Started "
                << cameras_ptr->clock->current_time().time_since_epoch().count()
                << "\n";
    }
  });

  tracker_ptr->AddKeyCallback("ResetVideo", [&](char key) {
    if (key == 'r') {
      cameras_ptr->clock->stop_at(reset_time ? *reset_time : cameras_ptr->clock->get_clock_min());
    }
  });

  // cameras_ptr->clock->stop_at(
  //     m3t::steady_time_point(std::chrono::nanoseconds(1705487275472000000)) +
  //     std::chrono::seconds(1));

  auto detector_ptr = std::dynamic_pointer_cast<m3t::StaticDetector>(
      *std::ranges::find_if(tracker_ptr->detector_ptrs(), [=](auto &d) {
        return d->name() == body_of_interest + "_detector";
      }));

  assert(detector_ptr);

  bool toggle_rotation = false;

  tracker_ptr->AddKeyCallback("Translator", [&](char key) {
    if (toggle_rotation) return;
    Eigen::Vector3f translation;
    switch (key) {
      case 'w':
        translation = Eigen::Vector3f(-0.0001, 0, 0);
        break;
      case 's':
        translation = Eigen::Vector3f(0.0001, 0, 0);
        break;
      case 'a':
        translation = Eigen::Vector3f(0, -0.0001, 0);
        break;
      case 'd':
        translation = Eigen::Vector3f(0, 0.0001, 0);
        break;
      case 'f':
        translation = Eigen::Vector3f(0, 0, -0.0001);
        break;
      case 'g':
        translation = Eigen::Vector3f(0, 0, 0.0001);
        break;
      default:
        return;
    }

    auto current = detector_ptr->link2world_pose();
    detector_ptr->set_link2world_pose(current.translate(translation));
    tracker_ptr->ExecuteDetection(false);
  });

  static constexpr float rad_1_degree = std::numbers::pi_v<float> / 180;

  tracker_ptr->AddKeyCallback("Rotator", [&](char key) {
    if (!toggle_rotation) return;
    Eigen::Matrix3f rotation;
    switch (key) {
      case 'w':
        rotation = Eigen::AngleAxisf(rad_1_degree, Eigen::Vector3f::UnitX());
        break;
      case 's':
        rotation = Eigen::AngleAxisf(-rad_1_degree, Eigen::Vector3f::UnitX());
        break;
      case 'a':
        rotation = Eigen::AngleAxisf(rad_1_degree, Eigen::Vector3f::UnitY());
        break;
      case 'd':
        rotation = Eigen::AngleAxisf(-rad_1_degree, Eigen::Vector3f::UnitY());
        break;
      case 'f':
        rotation = Eigen::AngleAxisf(rad_1_degree, Eigen::Vector3f::UnitZ());
        break;
      case 'g':
        rotation = Eigen::AngleAxisf(-rad_1_degree, Eigen::Vector3f::UnitZ());
        break;
      default:
        return;
    }

    auto current = detector_ptr->link2world_pose();
    detector_ptr->set_link2world_pose(current.rotate(rotation));
    tracker_ptr->ExecuteDetection(false);
  });

  tracker_ptr->AddKeyCallback("Toggle", [&](char key) {
    if (key == 'b') {
      toggle_rotation = !toggle_rotation;
      std::cout << detector_ptr->link2world_pose().matrix() << "\n";
    }
  });

  // Clock Update
  auto timestamps = m3t::getTimestampsFromConfig(base_directory / std::format("{}.yaml", "cot_spine"));
  {
    auto *sub = new m3t::LambdaSubscriber(
        "ClockUpdater", [&, clock = cameras_ptr->clock.get()](int i) {
          if (!started) return;
          auto index = std::distance(timestamps.begin(), std::ranges::lower_bound(timestamps, cameras_ptr->clock->current_time())) + 1;
          if (index < timestamps.size()) {
            clock->set_to(timestamps[index]);
          }
        });
    tracker_ptr->AddSubscriber(std::shared_ptr<m3t::Subscriber>{sub});
  }

  //{
  //  using namespace std::literals;
  //  auto *sub = new m3t::LambdaSubscriber(
  //      "ClockUpdater", [&, clock = cameras_ptr->clock.get()](int i) {
  //        if (!started) return;
  //        clock->set_to(clock->current_time() + 10ms);
  //      });
  //  tracker_ptr->AddSubscriber(std::shared_ptr<m3t::Subscriber>{sub});
  //}

  auto body_ptr = *std::ranges::find_if(renderer_geometry_ptr->body_ptrs(), [=](auto &d) {
    return d->name() == body_of_interest;
  });

  if constexpr (kLogPoses) {
    tracker_ptr->AddSubscriber(std::make_shared<m3t::PoseLogger>(
        cameras_ptr->clock.get(),
        body_ptr,
        std::filesystem::path(R"(C:\Users\leander.behr\OneDrive - Brainlab AG\IDP\Endoscope_Recordings\Evaluation_Red\Hook\VideoTrackingPoses_1x_test.txt)")));
  }

  // Start tracking
  if (!tracker_ptr->SetUp()) return -1;

  auto region_ptr = std::dynamic_pointer_cast<m3t::RegionModality>(
      *std::ranges::find_if(tracker_ptr->modality_ptrs(), [=](auto &d) {
        return d->name() == body_of_interest + "_region_modality";
      }));

  // auto texture_ptr = std::dynamic_pointer_cast<m3t::TextureModality>(
  //     *std::ranges::find_if(tracker_ptr->modality_ptrs(), [=](auto &d) {
  //       return d->name() == body_of_interest + "_texture_modality";
  //     }));

  tracker_ptr->AddKeyCallback("ToggleVis", [&](char key) {
    if (key == 'v') {
      region_ptr->set_visualize_lines_correspondence(!region_ptr->visualize_lines_correspondence());
      // region_ptr->set_visualize_gradient_optimization(!region_ptr->visualize_gradient_optimization());
      // region_ptr->set_visualize_hessian_optimization(!region_ptr->visualize_hessian_optimization());
      // region_ptr->set_visualize_points_optimization(!region_ptr->visualize_points_optimization());
      region_ptr->set_visualize_pose_result(!region_ptr->visualize_pose_result());
    }
  });

  tracker_ptr->AddKeyCallback("LogTip", [&, prev = Eigen::Vector3f()](char key) mutable {
    if (key == 'p') {
      auto pose = body_ptr->body2world_pose();
      auto tip = Eigen::Vector3f(0, 0.26037, 0);
      Eigen::Vector3f tip_world = pose * tip;
      auto dist = (tip_world - prev).norm();
      prev = tip_world;
      std::cout << "dist " << dist * 1000 << "\n";
      std::cout << tip_world.matrix() * 1000 << "\n\n";
    }
  });

  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  return 0;
}
