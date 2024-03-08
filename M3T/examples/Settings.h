#pragma once

#include <optional>
#include <vector>

struct ModalitySettings {
  std::optional<int> n_histogram_bins{};
  std::optional<std::vector<int>> scales{};
  std::optional<std::vector<float>> standard_deviations{};

  std::optional<float> max_considered_line_length{};
  std::optional<float> unconsidered_line_length{};

  std::optional<float> function_slope{};
  std::optional<float> function_amplitude{};

  std::optional<int> n_global_iterations{};
  std::optional<int> n_lines_max{};

  std::optional<float> min_continuous_distance{};
};

struct OptimizerSettings {
  std::optional<float> tikhonov_parameter_translation{};
  std::optional<float> tikhonov_parameter_rotation{};
};

struct TrackerSettings {
  std::optional<int> n_update_iterations;
};

struct Settings {
  ModalitySettings modality_settings;
  OptimizerSettings optimizer_settings;
  TrackerSettings tracker_settings;
};

auto gooder_hook_2_settings = Settings{
    .modality_settings = {
        .n_histogram_bins = 64,

        // 20s for big frame to frame jumps and recovery from losing tracking close to the image border
        .scales = std::vector{20, 20, 12, 12, 8},
        .standard_deviations = std::vector<float>{20, 20, 15, 15, 7},

        .max_considered_line_length = 200,
        .unconsidered_line_length = 5,

        .function_slope = 2,
        .function_amplitude = 0.3,

        .n_global_iterations = 10,
    },
    .optimizer_settings = {
        .tikhonov_parameter_translation = 500'000,
        .tikhonov_parameter_rotation = 1'000,
    },
    .tracker_settings = {
        .n_update_iterations = 10,
    },
};

auto good_hook_2_settings = Settings{
    .modality_settings = {
        .n_histogram_bins = 64,

        .scales = std::vector{12, 12, 8, 8, 4},
        .standard_deviations = std::vector<float>{15, 15, 7, 7, 4},

        .max_considered_line_length = 200,
        .unconsidered_line_length = 5,

        .function_slope = 2,
        .function_amplitude = 0.3,

        .n_global_iterations = 10,
    },
    .optimizer_settings = {
        .tikhonov_parameter_translation = 500'000,
        .tikhonov_parameter_rotation = 1'000,
    },
    .tracker_settings = {
        .n_update_iterations = 10,
    },
};