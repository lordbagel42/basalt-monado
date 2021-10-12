#include "slam_tracker.hpp"

#include <CLI/CLI.hpp>

#include <stdio.h>
#include <string>

#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/serialization/headers_serialization.h>

// TODO@mateosss: Reference in the readme that I'm compiling with
// cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DEIGEN_ROOT=/usr/include/eigen3
// cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DEIGEN_ROOT=/usr/include/eigen3

#define ASSERT(cond, ...)                                      \
  do {                                                         \
    if (!(cond)) {                                             \
      printf("Assertion failed @%s:%d\n", __func__, __LINE__); \
      printf(__VA_ARGS__);                                     \
      printf("\n");                                            \
      exit(EXIT_FAILURE);                                      \
    }                                                          \
  } while (false);
#define ASSERT_(cond) ASSERT(cond, "%s", #cond);

namespace xrt::auxiliary::tracking::slam {

using std::cout;
using std::string;

struct slam_tracker::implementation {
  // Options parsed from unified config file
  bool show_gui = true;
  string cam_calib_path;
  string dataset_path;
  string dataset_type;
  string config_path;
  string marg_data_path;
  bool print_queue = false;
  string result_path;
  int num_threads = 0;
  bool step_by_step = false;
  string trajectory_fmt;
  bool use_imu = true;

  // VIO members
  basalt::Calibration<double> calib;
  basalt::VioConfig vio_config;
  basalt::OpticalFlowBase::Ptr opt_flow_ptr;
  basalt::VioEstimatorBase::Ptr vio;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

  implementation(string unified_config) {
    // TODO@mateosss: [x] Argument parsing
    // TODO@mateosss: [x] tbb global control
    // TODO@mateosss: [x] vio_config load
    // TODO@mateosss: [x] load_data(cam_calib_path);
    // TODO@mateosss: [x] opt_flow_ptr
    // TODO@mateosss: [x] vio, initialize
    // TODO@mateosss: [x] setup queues
    // TODO@mateosss: [ ] t1: frame thread
    // TODO@mateosss: [ ] t2: imu thread
    // TODO@mateosss: [ ] t3: gui thread out_vis_queue
    // TODO@mateosss: [ ] t4: gui thread out_state_queue
    // TODO@mateosss: [ ] t5: print_queue thread
    // TODO@mateosss: [ ] gui setup
    // TODO@mateosss: [ ] joins

    load_unified_config(unified_config);

    // Asserts for the config file
    // TODO@mateosss: not sure how tbb::global_control affects std::threads
    ASSERT_(num_threads == 0);
    ASSERT(!config_path.empty(), "config-path missing from %s", unified_config.c_str());
    ASSERT_(use_imu);  // TODO@mateosss: Unsure if it works without this
    // TODO@mateosss: See what to do with this flag, it will just skip old
    // frames that weren't processed when a new one arrives
    ASSERT_(!vio_config.vio_enforce_realtime)

    vio_config.load(config_path);
    load_calibration_data(cam_calib_path);

    opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    vio = basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, use_imu);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    // if (show_gui) vio->out_vis_queue = &out_vis_queue; // TODO@mateosss: Use gui
    vio->out_state_queue = &out_state_queue;

    printf(">>> all good\n");
    exit(3);
  }

  void load_unified_config(const string &unified_config) {
    // TODO@mateosss: Check that the app stops when a required option is not present

    CLI::App app{"Options for the Basalt SLAM Tracker"};

    app.add_option("--show-gui", show_gui, "Show GUI");
    app.add_option("--cam-calib", cam_calib_path, "Ground-truth camera calibration used for simulation.")->required();
    app.add_option("--dataset-path", dataset_path, "Path to dataset.")->required();
    app.add_option("--dataset-type", dataset_type, "Dataset type <euroc, bag>.")->required();
    app.add_option("--config-path", config_path, "Path to config file.")->required();
    app.add_option("--marg-data", marg_data_path, "Path to folder where marginalization data will be stored.");
    app.add_option("--print-queue", print_queue, "Poll and print for queue sizes.");
    app.add_option("--result-path", result_path, "Path to result file where the system will write RMSE ATE.");
    app.add_option("--num-threads", num_threads, "Number of threads.");
    app.add_option("--step-by-step", step_by_step, "Manual playback.");
    app.add_option("--save-trajectory", trajectory_fmt, "Save if not empty. Supported formats <tum, euroc, kitti>");
    app.add_option("--use-imu", use_imu, "Use IMU.");

    try {
      // While --config-path sets the VIO configuration, --config sets the
      // entire unified Basalt configuration, including --config-path
      app.set_config("--config", unified_config, "Configuration file.", true);
      app.allow_config_extras(false);  // Makes parsing fail on unknown options
      string unique_config = "--config=" + unified_config;
      app.parse(unique_config);
    } catch (const CLI::ParseError &e) {
      app.exit(e);
    }

    cout << "Instantiating Basalt SLAM tracker\n";
    cout << "Using config file: " << app["--config"]->as<string>() << "\n";
    cout << app.config_to_str(true, true) << "\n";

    // TODO@mateosss: remove these asserts related to config file parsing
    ASSERT_(show_gui == true);
    ASSERT_(dataset_path ==
            "/home/mateo/Documents/apps/euroc/"
            "2_mateosss_20210924_640x360_30fps_stereo_acc250_gyr200/");
    ASSERT_(config_path == "/home/mateo/Documents/apps/bsltdeps/basalt/data/euroc_config.json");
  }

  void load_calibration_data(const std::string &calib_path) {
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

  void start() { printf(">>> implementation.%s\n", __func__); }

  void stop() { printf(">>> implementation.%s\n", __func__); }

  bool is_running() {
    printf(">>> implementation.%s\n", __func__);
    return false;
  }

  void push_imu_sample(imu_sample s) {
    (void)s;
    // TODO@mateosss: use this queue for IMU samples
    // t265_device->imu_data_queue = &vio->imu_data_queue;
    printf(">>> implementation.%s\n", __func__);
  }

  void push_frame(img_sample s) {
    (void)s;
    // TODO@mateosss use this queue for pushing frames
    // t265_device->image_data_queue = &opt_flow_ptr->input_queue;
    printf(">>> implementation.%s\n", __func__);
  }

  bool try_dequeue_pose(pose &pose) {
    (void)pose;
    printf(">>> implementation.%s\n", __func__);
    return false;
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
