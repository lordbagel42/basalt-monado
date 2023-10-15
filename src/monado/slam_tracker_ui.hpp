#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>

#include <sophus/se3.hpp>

#include <pangolin/display/display.h>
#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/pangolin.h>

#include <basalt/io/marg_data_io.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/keypoints.h>
#include <basalt/utils/vio_config.h>
#include <basalt/utils/vis_utils.h>
#include <basalt/vi_estimator/vio_estimator.h>

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
using namespace Eigen;

class slam_tracker_ui {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  VioVisualizationData::Ptr curr_vis_data = nullptr;
  VioVisualizationData::Ptr prev_vis_data = nullptr;
  pangolin::DataLog vio_data_log;
  thread vis_thread;
  thread ui_runner_thread;
  size_t num_cams = 0;
  std::atomic<bool> running = false;

 public:
  tbb::concurrent_bounded_queue<VioVisualizationData::Ptr> out_vis_queue{};
  tbb::concurrent_queue<double> *opt_flow_depth_queue = nullptr;
  OpticalFlowBase::Ptr opt_flow;

  void initialize(int ncams) {
    vio_data_log.Clear();
    ASSERT_(ncams > 0);
    num_cams = ncams;
  }

  void start(const Sophus::SE3d &T_w_i_init, const Calibration<double> &calib, const VioConfig &config,
             tbb::concurrent_queue<double> *ofq_depth, OpticalFlowBase::Ptr of) {
    opt_flow_depth_queue = ofq_depth;
    opt_flow = of;
    running = true;
    start_visualization_thread();
    start_ui(T_w_i_init, calib, config);
  }

  void stop() {
    running = false;
    vis_thread.join();
    ui_runner_thread.join();
  }

  void start_visualization_thread() {
    vis_thread = thread([&]() {
      while (true) {
        auto curr = curr_vis_data;
        out_vis_queue.pop(curr_vis_data);
        show_frame = show_frame + 1;
        if (curr_vis_data.get() == nullptr) break;
        prev_vis_data = curr;
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

    if (data->input_images->state_reset) {
      vio_t_ns.clear();
      vio_t_w_i.clear();
      vio_data_log.Clear();
    }

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
  Calibration<double> calib;
  VioConfig config;
  void start_ui(const Sophus::SE3d &T_w_i_init, const Calibration<double> &cal, const VioConfig &conf) {
    ui_runner_thread = thread(&slam_tracker_ui::ui_runner, this, T_w_i_init, cal, conf);
  }

  pangolin::Var<bool> follow{"ui.follow", true, false, true};
  pangolin::Var<bool> show_est_pos{"ui.show_est_pos", true, false, true};
  pangolin::Var<bool> show_est_vel{"ui.show_est_vel", false, false, true};
  pangolin::Var<bool> show_est_bg{"ui.show_est_bg", false, false, true};
  pangolin::Var<bool> show_est_ba{"ui.show_est_ba", false, false, true};

  void ui_runner(const Sophus::SE3d &T_w_i_init, const Calibration<double> &cal, const VioConfig &conf) {
    constexpr int UI_WIDTH = 200;

    calib = cal;
    config = conf;
    string window_name = "Basalt 6DoF Tracker for Monado";
    pangolin::CreateWindowAndBind(window_name, 1800, 1000);

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
      iv->UseNN() = true;  // Disable antialiasing (toggle it back with the N key)

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
        // TODO: There is a small race condition here over
        // curr_vis_data that is also present in the original basalt examples
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

      if (fixed_depth.GuiChanged() && opt_flow_depth_queue != nullptr) {
        opt_flow_depth_queue->push(fixed_depth);
      }
      depth_guess = opt_flow->depth_guess;

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

    pangolin::QuitAll();
    cout << "Finished ui_runner\n";
  }

  pangolin::Var<int> show_frame{"ui.show_frame", 0, pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> show_flow{"ui.show_flow", false, false, true};
  pangolin::Var<bool> show_tracking_guess{"ui.show_tracking_guess", false, false, true};
  pangolin::Var<bool> show_matching_guess{"ui.show_matching_guess", false, false, true};
  pangolin::Var<bool> show_recall_guess{"ui.show_recall_guess", true, false, true};
  pangolin::Var<bool> show_obs{"ui.show_obs", true, false, true};
  pangolin::Var<bool> show_ids{"ui.show_ids", false, false, true};
  pangolin::Var<bool> show_depth{"ui.show_depth", false, false, true};

  pangolin::Var<bool> show_grid{"ui.show_grid", false, false, true};
  pangolin::Var<bool> show_safe_radius{"ui.show_safe_radius", false, false, true};
  pangolin::Var<bool> show_cam0_proj{"ui.show_cam0_proj", false, false, true};
  pangolin::Var<bool> show_masks{"ui.show_masks", false, false, true};

  pangolin::Var<bool> show_guesses{"ui.Show matching guesses", false, false, true};
  pangolin::Var<bool> show_same_pixel_guess{"ui.SAME_PIXEL", true, false, true};
  pangolin::Var<bool> show_reproj_avg_depth_guess{"ui.REPROJ_AVG_DEPTH", true, false, true};
  pangolin::Var<bool> show_reproj_fix_depth_guess{"ui.REPROJ_FIX_DEPTH", true, false, true};
  pangolin::Var<double> fixed_depth{"ui.FIX_DEPTH", 2, 0, 3};
  pangolin::Var<bool> show_active_guess{"ui.Active Guess", true, false, true};

  pangolin::Var<double> depth_guess{"ui.depth_guess", 2, pangolin::META_FLAG_READONLY};

  void draw_image_overlay(pangolin::View &v, size_t cam_id) {
    UNUSED(v);

    if (!curr_vis_data ||                                               //
        !curr_vis_data->opt_flow_res ||                                 //
        !curr_vis_data->opt_flow_res->input_images ||                   //
        curr_vis_data->opt_flow_res->input_images->img_data.empty() ||  //
        !curr_vis_data->opt_flow_res->input_images->img_data.at(0).img) {
      return;
    }

    if (show_obs) {
      vis::show_obs(cam_id, curr_vis_data, config, calib, show_same_pixel_guess, show_reproj_fix_depth_guess,
                    show_reproj_avg_depth_guess, show_active_guess, fixed_depth, show_ids, show_depth, show_guesses);
    }

    if (show_flow) vis::show_flow(cam_id, curr_vis_data, opt_flow, show_ids);

    if (show_tracking_guess) vis::show_tracking_guess(cam_id, show_frame, curr_vis_data, prev_vis_data);

    if (show_matching_guess) vis::show_matching_guesses(cam_id, curr_vis_data);

    if (show_recall_guess) vis::show_recall_guesses(cam_id, curr_vis_data);

    if (show_masks) vis::show_masks(cam_id, curr_vis_data);

    if (show_cam0_proj) vis::show_cam0_proj(cam_id, depth_guess, config, calib);

    if (show_grid) vis::show_grid(config, calib);

    if (show_safe_radius) vis::show_safe_radius(config, calib);
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
}  // namespace xrt::auxiliary::tracking::slam
