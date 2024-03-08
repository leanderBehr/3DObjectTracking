#pragma once

#include <m3t/body.h>
#include <m3t/clock.h>
#include <m3t/subscriber.h>

#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>

namespace m3t {

class PoseLogger : public Subscriber {
 public:
  PoseLogger(Clock* clock, std::shared_ptr<Body> body_ptr, std::filesystem::path out_path) : Subscriber(std::format("PoseLogger_{}", body_ptr->name())),
                                                                                             clock_(clock),
                                                                                             body_ptr_(body_ptr),
                                                                                             output_() {
    if (std::filesystem::exists(out_path)) {
      std::cerr << "PoseLogger output file exists already!\n";
      throw std::exception("PoseLogger output file exists already!");
    }

    output_ = std::ofstream(out_path);
  }

  bool SetUp() override { return true; }
  bool UpdateSubscriber(int iteration) override {
    auto time = clock_->current_time();
    auto pose = body_ptr_->body2world_pose();

    output_ << "{\n  \"nanos\": " << time.time_since_epoch().count() << ",\n  \"matrix\":\n" << pose.matrix().format(matrix_fmt_) << "\n"
            << "}\n"
            << "---\n";
    return true;
  }

 private:
  Eigen::IOFormat matrix_fmt_{Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]"};

  Clock* clock_;
  std::shared_ptr<Body> body_ptr_;
  std::ofstream output_;
};

}  // namespace m3t