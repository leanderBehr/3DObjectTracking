// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem>
#include "m3t/cameras/opencv_camera.h"
#include <m3t/image_viewer.h>
#include <m3t/tracker.h>

#include <memory>
#include <string>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Not enough arguments: Provide sequence directory";
        return -1;
    }
    const std::filesystem::path sequence_directory{argv[1]};

    auto color_camera_ptr{
            std::make_shared<m3t::OpenCVCamera>("color_camera", m3t::Intrinsics{
                    .fu=619.65f,
                    .fv=623.52f,
                    .ppu=315.73f,
                    .ppv=231.98f,
                    .width=640,
                    .height=480
            }, std::vector{-0.09060528, 0.86039354, 0.00231482, 0.00488739, -2.11455125})};

    color_camera_ptr->StartSavingImages(sequence_directory);

    auto color_viewer_ptr{std::make_shared<m3t::ImageColorViewer>(
            "color_viewer", color_camera_ptr)};

    auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
    tracker_ptr->AddViewer(color_viewer_ptr);
    if (!tracker_ptr->SetUp()) return -1;
    if (!tracker_ptr->RunTrackerProcess(false, false)) return -1;
    return 0;
}
