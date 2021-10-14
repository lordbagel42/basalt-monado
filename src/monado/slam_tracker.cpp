// Copyright 2021, Collabora, Ltd.

#include "slam_tracker.hpp"

#include <pangolin/pangolin.h>
#include <CLI/CLI.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/vi_estimator/vio_estimator.h>
// TODO@mateosss: Reference in the readme that I'm compiling with
// cmake . -B build -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo
// -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DEIGEN_ROOT=/usr/include/eigen3

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

class slam_tracker_ui {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  VioVisualizationData::Ptr xxxcurr_vis_data;
  pangolin::DataLog xxxvio_data_log;
  thread vis_thread;

 public:
  tbb::concurrent_bounded_queue<VioVisualizationData::Ptr> out_vis_queue{};

  void initialize() { xxxvio_data_log.Clear(); }

  void start_visualization_thread() {
    vis_thread = thread([&]() {
      while (true) {
        out_vis_queue.pop(xxxcurr_vis_data);
        if (xxxcurr_vis_data.get() == nullptr) break;
      }
      cout << "Finished vis_thread\n";
    });
  }

  void log_vio_data(const PoseVelBiasState<double>::Ptr &data, float since_start_ns) {
    Sophus::SE3d T_w_i = data->T_w_i;
    Eigen::Vector3d vel_w_i = data->vel_w_i;
    Eigen::Vector3d bg = data->bias_gyro;
    Eigen::Vector3d ba = data->bias_accel;

    vector<float> vals;
    vals.push_back(since_start_ns * 1e-9);
    for (int i = 0; i < 3; i++) vals.push_back(vel_w_i[i]);
    for (int i = 0; i < 3; i++) vals.push_back(T_w_i.translation()[i]);
    for (int i = 0; i < 3; i++) vals.push_back(bg[i]);
    for (int i = 0; i < 3; i++) vals.push_back(ba[i]);

    xxxvio_data_log.Log(vals);
  }

  void stop() { vis_thread.join(); }
};

struct slam_tracker::implementation {
 private:
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
  Calibration<double> calib;
  VioConfig vio_config;
  OpticalFlowBase::Ptr opt_flow_ptr;
  VioEstimatorBase::Ptr vio;

  // Queues
  std::atomic<bool> running = false;
  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr> out_state_queue;
  PoseVelBiasState<double> last_out_state{};  // TODO@mateosss: put a lock when writting this
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
    // TODO@mateosss: [ ] Check that there are no XXX comments floating around
    // TODO@mateosss: [ ] Check that there are no xxxmembers floating around

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

    opt_flow_ptr = OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    image_data_queue = &opt_flow_ptr->input_queue;
    ASSERT_(image_data_queue != nullptr);

    vio = VioEstimatorFactory::getVioEstimator(vio_config, calib, constants::g, use_imu);
    imu_data_queue = &vio->imu_data_queue;
    ASSERT_(imu_data_queue != nullptr);

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    if (show_gui) {
      ui.initialize();
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

  int64_t start_t_ns = -1;
  vector<int64_t> xxxvio_t_ns;
  Eigen::aligned_vector<Eigen::Vector3d> xxxvio_t_w_i;

  void state_consumer() {
    PoseVelBiasState<double>::Ptr data;

    while (true) {
      out_state_queue.pop(data);
      if (data.get() == nullptr) {
        monado_out_state_queue.push(nullptr);
        break;
      }

      last_out_state = *data;

      int64_t t_ns = data->t_ns;
      if (start_t_ns < 0) start_t_ns = t_ns;

      xxxvio_t_ns.emplace_back(t_ns);
      xxxvio_t_w_i.emplace_back(data->T_w_i.translation());

      if (show_gui) ui.log_vio_data(data, t_ns - start_t_ns);

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

    if (show_gui) ui.start_visualization_thread();
    state_consumer_thread = thread(&slam_tracker::implementation::state_consumer, this);
    if (print_queue) queues_printer_thread = thread(&slam_tracker::implementation::queues_printer, this);
    // TODO@mateosss: if (show_gui) do stuff
  }

  void stop() {
    // Pushing nullptr signals end of stream
    // TODO@mateosss: could it be a race condition here?
    running = false;
    image_data_queue->push(nullptr);
    imu_data_queue->push(nullptr);

    if (print_queue) queues_printer_thread.join();
    state_consumer_thread.join();
    if (show_gui) ui.stop();

    // TODO@mateosss: There is a segfault when closing monado without starting the stream
    // happens in a lambda from keypoint_vio.cpp and ends at line calib_bias.hpp:112
  }

  bool is_running() {
    // TODO@mateosss: implement
    printf(">>> implementation.%s\n", __func__);
    return false;
  }

  void push_imu_sample(imu_sample s) {
    // TODO@mateosss: remove print
    // printf(">>> %s\n", imu2str(s).c_str());

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
    // TODO@mateosss: remove print
    // printf(">>> %s\n", img2str(s).c_str());

    int i = -1;
    if (s.is_left) {               // TODO@mateosss: Support mono?
      constexpr int NUM_CAMS = 2;  // TODO@mateosss: add to config file?
      partial_frame = make_shared<OpticalFlowInput>();
      partial_frame->img_data.resize(NUM_CAMS);
      partial_frame->t_ns = s.timestamp;
      i = 0;
    } else {
      ASSERT(partial_frame->t_ns == s.timestamp, "Left and right frame timestamps differ: %ld != %ld",
             partial_frame->t_ns, s.timestamp);
      i = 1;
    }

    // TODO@mateosss: exposure?
    // data->img_data[i].exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-6;

    int width = s.img.cols;
    int height = s.img.rows;
    // TODO@mateosss: why is it uint16_t instead of uint8_t?
    auto &mimg = partial_frame->img_data[i].img;
    mimg.reset(new ManagedImage<uint16_t>(width, height));

    // TODO@mateosss: We should avoid this copy. See if ManagedImage can use a cv Mat somehow
    size_t full_size = width * height;
    for (size_t j = 0; j < full_size; j++) {
      mimg->ptr[j] = s.img.data[j] << 8;
    }

    if (!s.is_left) {
      image_data_queue->push(partial_frame);
    }
  }

  bool try_dequeue_pose(pose &pose) {
// TODO@mateosss: USE_LAST_POSE has race conditions and honestly this is kind of ugly
// if USE_LAST_POSE is false, we will dequeue every pose produced by the VIO estimator
#define USE_LAST_POSE true
#if USE_LAST_POSE
    PoseVelBiasState<double> data = last_out_state;
    Sophus::SE3d T_w_i = data.T_w_i;
    pose.px = T_w_i.translation().x();
    pose.py = T_w_i.translation().y();
    pose.pz = T_w_i.translation().z();
    pose.rx = T_w_i.unit_quaternion().x();
    pose.ry = T_w_i.unit_quaternion().y();
    pose.rz = T_w_i.unit_quaternion().z();
    pose.rw = T_w_i.unit_quaternion().w();
    // TODO@mateosss: remove print
    printf(">>> try_dequeue_pose %s\n", pose2str(pose).c_str());

    return true;
#else
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
      // TODO@mateosss: remove print
      printf(">>> try_dequeue_pose %s\n", pose2str(pose).c_str());
    }
    return dequeued;
#endif
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
