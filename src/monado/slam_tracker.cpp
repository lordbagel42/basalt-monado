// Copyright 2021, Collabora, Ltd.

#include "slam_tracker.hpp"

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
  VioVisualizationData::Ptr curr_vis_data;
  pangolin::DataLog vio_data_log;
  thread vis_thread;
  thread ui_runner_thread;
  size_t num_cams = 0;
  std::atomic<bool> running = false;

 public:
  tbb::concurrent_bounded_queue<VioVisualizationData::Ptr> out_vis_queue{};

  void initialize(int ncams) {
    vio_data_log.Clear();
    ASSERT_(ncams > 0);
    num_cams = ncams;
  }

  void start(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &calib) {
    running = true;
    start_visualization_thread();
    start_ui(T_w_i_init, calib);
  }

  void stop() {
    running = false;
    vis_thread.join();
    ui_runner_thread.join();
  }

  void start_visualization_thread() {
    vis_thread = thread([&]() {
      while (true) {
        out_vis_queue.pop(curr_vis_data);
        if (curr_vis_data.get() == nullptr) break;
      }
      cout << "Finished vis_thread\n";
    });
  }

  int64_t start_t_ns = -1;
  std::vector<int64_t> vio_t_ns;
  Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;

  void log_vio_data(const PoseVelBiasState<double>::Ptr &data) {
    int64_t t_ns = data->t_ns;
    if (start_t_ns < 0) start_t_ns = t_ns;
    float since_start_ns = t_ns - start_t_ns;

    vio_t_ns.emplace_back(t_ns);
    vio_t_w_i.emplace_back(data->T_w_i.translation());

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

    vio_data_log.Log(vals);
  }

  pangolin::Plotter *plotter;
  pangolin::DataLog imu_data_log{};
  basalt::Calibration<double> calib;
  void start_ui(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &c) {
    ui_runner_thread = thread(&slam_tracker_ui::ui_runner, this, T_w_i_init, c);
  }

  pangolin::Var<bool> follow{"ui.follow", true, false, true};
  pangolin::Var<bool> show_est_pos{"ui.show_est_pos", true, false, true};
  pangolin::Var<bool> show_est_vel{"ui.show_est_vel", false, false, true};
  pangolin::Var<bool> show_est_bg{"ui.show_est_bg", false, false, true};
  pangolin::Var<bool> show_est_ba{"ui.show_est_ba", false, false, true};

  void ui_runner(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &c) {
    constexpr int UI_WIDTH = 200;

    calib = c;
    pangolin::CreateWindowAndBind("Basalt SLAM Tracker for Monado", 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    pangolin::View &img_view_display = pangolin::CreateDisplay()
                                           .SetBounds(0.4, 1.0, pangolin::Attach::Pix(UI_WIDTH), 0.4)
                                           .SetLayout(pangolin::LayoutEqual);

    pangolin::View &plot_display = pangolin::CreateDisplay().SetBounds(0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100, -3.0, 3.0, 0.01, 0.01);
    plot_display.AddDisplay(*plotter);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    while (img_view.size() < calib.intrinsics.size()) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display.AddDisplay(*iv);
      iv->extern_draw_function = [idx, this](auto &v) { this->draw_image_overlay(v, idx); };
    }

    Eigen::Vector3d cam_p(0.5, -2, -2);
    cam_p = T_w_i_init.so3() * calib.T_i_c[0].so3() * cam_p;
    cam_p[2] = 1;

    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0, pangolin::AxisZ));

    pangolin::View &display3D = pangolin::CreateDisplay()
                                    .SetAspect(-640 / 480.0)
                                    .SetBounds(0.4, 1.0, 0.4, 1.0)
                                    .SetHandler(new pangolin::Handler3D(camera));

    while (running && !pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (follow) {
        // TODO@mateosss: Pretty sure we have a race condition here over
        // curr_vis_data, but they had it in the original code as well
        if (curr_vis_data) {
          auto T_w_i = curr_vis_data->states.back();
          T_w_i.so3() = Sophus::SO3d();

          camera.Follow(T_w_i.matrix());
        }
      }

      display3D.Activate(camera);
      glClearColor(1.0, 1.0, 1.0, 1.0);

      draw_scene();

      img_view_display.Activate();

      {
        pangolin::GlPixFormat fmt;
        fmt.glformat = GL_LUMINANCE;
        fmt.gltype = GL_UNSIGNED_SHORT;
        fmt.scalable_internal_format = GL_LUMINANCE16;

        if (curr_vis_data && curr_vis_data->opt_flow_res && curr_vis_data->opt_flow_res->input_images) {
          auto &img_data = curr_vis_data->opt_flow_res->input_images->img_data;

          for (size_t cam_id = 0; cam_id < num_cams; cam_id++) {
            if (img_data[cam_id].img) {
              img_view[cam_id]->SetImage(img_data[cam_id].img->ptr, img_data[cam_id].img->w, img_data[cam_id].img->h,
                                         img_data[cam_id].img->pitch, fmt);
            }
          }
        }

        draw_plots();
      }

      if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() || show_est_ba.GuiChanged() ||
          show_est_bg.GuiChanged()) {
        draw_plots();
      }

      pangolin::FinishFrame();
    }

    cout << "Finished ui_runner\n";
  }

  pangolin::Var<bool> show_obs{"ui.show_obs", true, false, true};
  pangolin::Var<bool> show_ids{"ui.show_ids", false, false, true};

  void draw_image_overlay(pangolin::View &v, size_t cam_id) {
    UNUSED(v);

    if (show_obs) {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 0.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (curr_vis_data && cam_id < curr_vis_data->projections.size()) {
        const auto &points = curr_vis_data->projections[cam_id];

        if (!points.empty()) {
          double min_id = points[0][2];
          double max_id = points[0][2];

          for (const auto &points2 : curr_vis_data->projections) {
            for (const auto &p : points2) {
              min_id = std::min(min_id, p[2]);
              max_id = std::max(max_id, p[2]);
            }
          }

          for (const auto &c : points) {
            const float radius = 6.5;

            float r;
            float g;
            float b;
            getcolor(c[2] - min_id, max_id - min_id, b, g, r);
            glColor3f(r, g, b);

            pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

            if (show_ids) pangolin::GlFont::I().Text("%d", int(c[3])).Draw(c[0], c[1]);
          }
        }

        glColor3f(1.0, 0.0, 0.0);
        pangolin::GlFont::I().Text("Tracked %d points", points.size()).Draw(5, 20);
      }
    }
  }

  void draw_scene() {
    glPointSize(3);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor3ubv(cam_color);
    Eigen::aligned_vector<Eigen::Vector3d> sub_gt(vio_t_w_i.begin(), vio_t_w_i.end());
    pangolin::glDrawLineStrip(sub_gt);

    if (curr_vis_data) {
      for (const auto &p : curr_vis_data->states)
        for (const auto &t_i_c : calib.T_i_c) render_camera((p * t_i_c).matrix(), 2.0, state_color, 0.1);

      for (const auto &p : curr_vis_data->frames)
        for (const auto &t_i_c : calib.T_i_c) render_camera((p * t_i_c).matrix(), 2.0, pose_color, 0.1);

      for (const auto &t_i_c : calib.T_i_c)
        render_camera((curr_vis_data->states.back() * t_i_c).matrix(), 2.0, cam_color, 0.1);

      glColor3ubv(pose_color);
      pangolin::glDrawPoints(curr_vis_data->points);
    }

    pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
  }

  OpticalFlowInput::Ptr last_img_data{};
  void update_last_image(OpticalFlowInput::Ptr &data) { last_img_data = data; }
  void draw_plots() {
    plotter->ClearSeries();
    plotter->ClearMarkers();

    if (show_est_pos) {
      plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine, pangolin::Colour::Red(), "position x", &vio_data_log);
      plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine, pangolin::Colour::Green(), "position y", &vio_data_log);
      plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "position z", &vio_data_log);
    }

    if (show_est_vel) {
      plotter->AddSeries("$0", "$1", pangolin::DrawingModeLine, pangolin::Colour::Red(), "velocity x", &vio_data_log);
      plotter->AddSeries("$0", "$2", pangolin::DrawingModeLine, pangolin::Colour::Green(), "velocity y", &vio_data_log);
      plotter->AddSeries("$0", "$3", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "velocity z", &vio_data_log);
    }

    if (show_est_bg) {
      plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine, pangolin::Colour::Red(), "gyro bias x", &vio_data_log);
      plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine, pangolin::Colour::Green(), "gyro bias y",
                         &vio_data_log);
      plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
    }

    if (show_est_ba) {
      plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine, pangolin::Colour::Red(), "accel bias x",
                         &vio_data_log);
      plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine, pangolin::Colour::Green(), "accel bias y",
                         &vio_data_log);
      plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "accel bias z",
                         &vio_data_log);
    }

    if (last_img_data) {
      double t = last_img_data->t_ns * 1e-9;
      plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal, pangolin::Colour::White());
    }
  }
};

struct slam_tracker::implementation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  static constexpr int NUM_CAMS = 2;  // TODO@mateosss: add to config file?

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
  implementation(const string &unified_config) {
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

  void state_consumer() {
    PoseVelBiasState<double>::Ptr data;

    while (true) {
      out_state_queue.pop(data);
      if (data.get() == nullptr) {
        monado_out_state_queue.push(nullptr);
        break;
      }

      if (show_gui) ui.log_vio_data(data);
      last_out_state = *data;
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
    // TODO@mateosss: Prefix all prints and cout<<s with [Basalt]
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
    if (s.is_left) {  // TODO@mateosss: Support mono?
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
      if (show_gui) ui.update_last_image(partial_frame);
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
