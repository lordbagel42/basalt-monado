#include "slam_tracker.hpp"

#include <stdio.h>
#include <string>

namespace xrt::auxiliary::tracking::slam {

using std::string;

struct slam_tracker::implementation {
  implementation(string config_file) {
    (void)config_file;
    printf(">>> implementation.%s\n", __func__);
  }

  void start() { printf(">>> implementation.%s\n", __func__); }

  void stop() { printf(">>> implementation.%s\n", __func__); }

  bool is_running() {
    printf(">>> implementation.%s\n", __func__);
    return false;
  }

  void push_imu_sample(imu_sample s) {
    (void)s;
    printf(">>> implementation.%s\n", __func__);
  }

  void push_frame(img_sample s) {
    (void)s;
    printf(">>> implementation.%s\n", __func__);
  }

  bool try_dequeue_pose(pose &pose) {
    (void)pose;
    printf(">>> implementation.%s\n", __func__);
    return false;
  }
};

slam_tracker::slam_tracker(string config_file) {
  impl = new slam_tracker::implementation{config_file};
}

slam_tracker::~slam_tracker() { delete impl; }

void slam_tracker::start() { impl->start(); }

void slam_tracker::stop() { impl->stop(); }

bool slam_tracker::is_running() { return impl->is_running(); }

void slam_tracker::push_imu_sample(imu_sample s) { impl->push_imu_sample(s); }

void slam_tracker::push_frame(img_sample sample) { impl->push_frame(sample); }

bool slam_tracker::try_dequeue_pose(pose &pose) {
  return impl->try_dequeue_pose(pose);
}

}  // namespace xrt::auxiliary::tracking::slam
