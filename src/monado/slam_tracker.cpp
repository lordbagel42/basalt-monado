// Copyright 2021, Collabora, Ltd.

#include "slam_tracker.hpp"
#include "slam_tracker_ui.hpp"

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include "sophus/se3.hpp"

#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include "basalt/utils/vis_utils.h"

// TODO@mateosss: Reference in the readme that I'm compiling with
// cmake . -B build -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo
// -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DEIGEN_ROOT=/usr/include/eigen3 -DBUILD_TESTS=OFF

namespace xrt::auxiliary::tracking::slam {

using std::cout;
using std::make_shared;
using std::shared_ptr;
using std::string;
using std::thread;
using std::to_string;
using std::vector;
using namespace basalt;

string imu2str(const imu_sample &s) {
  string str = "imu_sample ";
  str += "t=" + to_string(s.timestamp) + " ";
  str += "a=[" + to_string(s.ax) + ", " + to_string(s.ay) + ", " + to_string(s.az) + "] ";
  str += "w=[" + to_string(s.wx) + ", " + to_string(s.wy) + ", " + to_string(s.wz) + "]";
  return str;
}

string img2str(const img_sample &s) {
  string str = "img_sample ";
  str += s.is_left ? "left " : "right ";
  str += "t=" + to_string(s.timestamp);
  return str;
}

string pose2str(const pose &p) {
  string str = "pose ";
  str += "p=[" + to_string(p.px) + ", " + to_string(p.py) + ", " + to_string(p.pz) + "] ";
  str += "r=[" + to_string(p.rx) + ", " + to_string(p.ry) + ", " + to_string(p.rz) + ", " + to_string(p.rw) + "]";
  return str;
}


struct slam_tracker::implementation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Options parsed from unified config file
  bool show_gui = true;
  string cam_calib_path;
  string config_path;
  string marg_data_path;
  bool print_queue = false;
  bool latest_pose = true;

  // Basalt in its current state does not support monocular cameras, although it
  // should be possible to adapt it to do so, see:
  // https://gitlab.com/VladyslavUsenko/basalt/-/issues/2#note_201965760
  // https://gitlab.com/VladyslavUsenko/basalt/-/issues/25#note_362741510
  // https://github.com/DLR-RM/granite
  static constexpr int NUM_CAMS = 2;

  // VIO members
  Calibration<double> calib;
  VioConfig vio_config;
  OpticalFlowBase::Ptr opt_flow_ptr;
  VioEstimatorBase::Ptr vio;
  bool expecting_left_frame = true;

  // Queues
  std::atomic<bool> running = false;
  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr> out_state_queue;
  shared_ptr<PoseVelBiasState<double>> last_out_state{};
  std::mutex last_out_state_lock{};  // atomic_shared_ptr is not standard, and atomic<shared_ptr> since c++20 only :/
  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr> monado_out_state_queue;
  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr> *image_data_queue = nullptr;  // Invariant: not null after ctor
  tbb::concurrent_bounded_queue<ImuData<double>::Ptr> *imu_data_queue = nullptr;     // Invariant: not null after ctor

  // Threads
  thread state_consumer_thread;
  thread queues_printer_thread;

  // External Queues
  slam_tracker_ui ui{};
  MargDataSaver::Ptr marg_data_saver;

 public:
  implementation(const string &unified_config) {
    load_unified_config(unified_config);

    vio_config.load(config_path);
    load_calibration_data(cam_calib_path);

    opt_flow_ptr = OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    image_data_queue = &opt_flow_ptr->input_queue;
    ASSERT_(image_data_queue != nullptr);

    vio = VioEstimatorFactory::getVioEstimator(vio_config, calib, constants::g, true);
    imu_data_queue = &vio->imu_data_queue;
    ASSERT_(imu_data_queue != nullptr);

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    if (show_gui) {
      ui.initialize(NUM_CAMS);
      vio->out_vis_queue = &ui.out_vis_queue;
    };
    vio->out_state_queue = &out_state_queue;

    if (!marg_data_path.empty()) {
      marg_data_saver.reset(new MargDataSaver(marg_data_path));
      vio->out_marg_queue = &marg_data_saver->in_marg_queue;
    }
  }

 private:
  void load_unified_config(const string &unified_config) {
    CLI::App app{"Options for the Basalt SLAM Tracker"};

    app.add_option("--show-gui", show_gui, "Show GUI");
    app.add_option("--cam-calib", cam_calib_path, "Ground-truth camera calibration used for simulation.")->required();
    app.add_option("--config-path", config_path, "Path to config file.")->required();
    app.add_option("--marg-data", marg_data_path, "Path to folder where marginalization data will be stored.");
    app.add_option("--print-queue", print_queue, "Poll and print for queue sizes.");
    app.add_option("--latest-pose", latest_pose,
                   "0 for returning every tracked pose in order, 1 (default) for just the latest");

    try {
      // While --config-path sets the VIO configuration, --config sets the
      // entire unified Basalt configuration, including --config-path
      app.set_config("--config", unified_config, "Configuration file.", true);
      app.allow_config_extras(false);  // Makes parsing fail on unknown options
      string unique_config = "--config=" + unified_config;
      app.parse(unique_config);
    } catch (const CLI::ParseError &e) {
      app.exit(e);
      ASSERT(false, "Config file error (%s)", unified_config.c_str());
    }

    cout << "Instantiating Basalt SLAM tracker\n";
    cout << "Using config file: " << app["--config"]->as<string>() << "\n";
    cout << app.config_to_str(true, true) << "\n";
  }

  void load_calibration_data(const string &calib_path) {
    std::ifstream os(calib_path, std::ios::binary);
    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib);
      std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras" << std::endl;
    } else {
      std::cerr << "could not load camera calibration " << calib_path << std::endl;
      std::abort();
    }
  }

  void state_consumer() {
    PoseVelBiasState<double>::Ptr data;

    while (true) {
      out_state_queue.pop(data);
      if (data.get() == nullptr) {
        monado_out_state_queue.push(nullptr);
        break;
      }

      if (show_gui) ui.log_vio_data(data);

      last_out_state_lock.lock();
      last_out_state = data;
      last_out_state_lock.unlock();

      monado_out_state_queue.push(data);
    }

    std::cout << "Finished state_consumer" << std::endl;
  }

  void queues_printer() {
    while (running) {
      cout << "opt_flow_ptr->input_queue " << opt_flow_ptr->input_queue.size() << " opt_flow_ptr->output_queue "
           << opt_flow_ptr->output_queue->size() << " out_state_queue " << out_state_queue.size() << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Finished queues_printer" << std::endl;
  }

 public:
  void start() {
    running = true;
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (show_gui) ui.start(vio->getT_w_i_init(), calib);
    state_consumer_thread = thread(&slam_tracker::implementation::state_consumer, this);
    if (print_queue) queues_printer_thread = thread(&slam_tracker::implementation::queues_printer, this);
  }

  void stop() {
    running = false;
    image_data_queue->push(nullptr);
    imu_data_queue->push(nullptr);

    if (print_queue) queues_printer_thread.join();
    state_consumer_thread.join();
    if (show_gui) ui.stop();

    // TODO: There is a segfault when closing monado without starting the stream
    // happens in a lambda from keypoint_vio.cpp and ends at line calib_bias.hpp:112
  }

  bool is_running() { return running; }

  void push_imu_sample(imu_sample s) {
    // concurrent_bounded_queue expects Erasable and Allocator named
    // requirements for the type, using a pointer because it already is. This is
    // done in the others examples as well but it is far from optimal.
    ImuData<double>::Ptr data;
    data.reset(new ImuData<double>);
    data->t_ns = s.timestamp;
    data->accel = {s.ax, s.ay, s.az};
    data->gyro = {s.wx, s.wy, s.wz};
    imu_data_queue->push(data);
  }

 private:
  OpticalFlowInput::Ptr partial_frame;

 public:
  void push_frame(img_sample s) {
    ASSERT(expecting_left_frame == s.is_left, "Unexpected %s frame", s.is_left ? "left" : "right");
    expecting_left_frame = !expecting_left_frame;

    int i = -1;
    if (s.is_left) {
      partial_frame = make_shared<OpticalFlowInput>();
      partial_frame->img_data.resize(NUM_CAMS);
      partial_frame->t_ns = s.timestamp;
      i = 0;
    } else {
      ASSERT(partial_frame->t_ns == s.timestamp, "Left and right frame timestamps differ: %ld != %ld",
             partial_frame->t_ns, s.timestamp);
      i = 1;
    }

    int width = s.img.cols;
    int height = s.img.rows;
    // Forced to use uint16_t here, in place because of cameras with 12-bit grayscale support
    auto &mimg = partial_frame->img_data[i].img;
    mimg.reset(new ManagedImage<uint16_t>(width, height));

    // TODO: We could avoid this copy. Maybe by writing a custom
    // allocator for ManagedImage that ties the OpenCV allocator
    size_t full_size = width * height;
    for (size_t j = 0; j < full_size; j++) {
      mimg->ptr[j] = s.img.data[j] << 8;
    }

    if (!s.is_left) {
      image_data_queue->push(partial_frame);
      if (show_gui) ui.update_last_image(partial_frame);
    }
  }

  bool try_dequeue_pose(pose &pose) {
    if (latest_pose) {  // Return only the last produced pose
      last_out_state_lock.lock();
      Sophus::SE3d T_w_i = last_out_state->T_w_i;
      last_out_state_lock.unlock();
      pose.px = T_w_i.translation().x();
      pose.py = T_w_i.translation().y();
      pose.pz = T_w_i.translation().z();
      pose.rx = T_w_i.unit_quaternion().x();
      pose.ry = T_w_i.unit_quaternion().y();
      pose.rz = T_w_i.unit_quaternion().z();
      pose.rw = T_w_i.unit_quaternion().w();
      return true;
    } else {  // Return every pose produced by the VIO estimator
      PoseVelBiasState<double>::Ptr data;
      bool dequeued = monado_out_state_queue.try_pop(data);
      if (dequeued) {
        Sophus::SE3d T_w_i = data->T_w_i;
        pose.px = T_w_i.translation().x();
        pose.py = T_w_i.translation().y();
        pose.pz = T_w_i.translation().z();
        pose.rx = T_w_i.unit_quaternion().x();
        pose.ry = T_w_i.unit_quaternion().y();
        pose.rz = T_w_i.unit_quaternion().z();
        pose.rw = T_w_i.unit_quaternion().w();
      }
      return dequeued;
    }
  }
};

slam_tracker::slam_tracker(string config_file) { impl = new slam_tracker::implementation{config_file}; }

slam_tracker::~slam_tracker() { delete impl; }

void slam_tracker::start() { impl->start(); }

void slam_tracker::stop() { impl->stop(); }

bool slam_tracker::is_running() { return impl->is_running(); }

void slam_tracker::push_imu_sample(imu_sample s) { impl->push_imu_sample(s); }

void slam_tracker::push_frame(img_sample sample) { impl->push_frame(sample); }

bool slam_tracker::try_dequeue_pose(pose &pose) { return impl->try_dequeue_pose(pose); }

}  // namespace xrt::auxiliary::tracking::slam
