/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2023, Collabora Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/vis_utils.h>
#include <pangolin/gl/glfont.h>
#include <pangolin/var/var.h>

namespace pangolin {
extern "C" const unsigned char AnonymousPro_ttf[];
}

namespace basalt::vis {
using UIHessians = VioVisualizationData::UIHessians;
using UIJacobians = VioVisualizationData::UIJacobians;
auto is_hessian = VioVisualizationData::is_hessian;
auto is_jacobian = VioVisualizationData::is_jacobian;

pangolin::GlFont SMALL_FONT(pangolin::AnonymousPro_ttf, 11);

bool try_draw_image_text(pangolin::ImageView& view, float x, float y, const pangolin::GlText& text) {
  float xwin = -1;
  float ywin = -1;
  view.ImageToScreen(view.v, x, y, xwin, ywin);

  bool in_bounds = view.GetBounds().Contains(xwin, ywin);
  if (in_bounds) text.Draw(x, y);
  return in_bounds;
}

bool VIOUIBase::highligh_frame() {
  VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return false;

  std::set<KeypointId> kpids;
  for (const auto& kps : get_curr_vis_data()->opt_flow_res->keypoints)
    for (const auto& [kpid, kp] : kps) kpids.insert(kpid);

  std::string str = highlight_landmarks;
  str.reserve(str.size() + kpids.size() * 10);
  for (const KeypointId& kpid : kpids) str += std::to_string(kpid) + ",";
  if (!str.empty()) str.pop_back();

  highlight_landmarks = str;
  highlight_landmarks.Meta().gui_changed = true;

  return true;
}

bool VIOUIBase::toggle_blocks() {
  show_blocks = do_toggle_blocks(blocks_display, plot_display, img_view_display, UI_WIDTH);
  return show_blocks;
}

void VIOUIBase::do_show_flow(size_t cam_id, const OpticalFlowBase::Ptr& opt_flow) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  pangolin::ImageView& view = *img_view.at(cam_id);

  glLineWidth(1.0);
  glColor3ubv(RED);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const std::vector<Keypoints>& keypoints = curr_vis_data->opt_flow_res->keypoints;
  if (cam_id >= keypoints.size()) return;

  const Keypoints& kp_map = keypoints[cam_id];
  const KeypointResponses& responses = curr_vis_data->opt_flow_res->keypoint_responses[cam_id];

  for (const auto& [lmid, pose] : kp_map) {
    bool show = !filter_highlights || is_selected(highlights, lmid);
    if (!show) continue;

    MatrixXf transformed_patch = pose.linear() * opt_flow->patch_coord;
    transformed_patch.colwise() += pose.translation();

    for (int i = 0; i < transformed_patch.cols(); i++) {
      const Vector2f c = transformed_patch.col(i);
      pangolin::glDrawCirclePerimeter(c[0], c[1], 0.5F);
    }

    const Vector2f c = pose.translation();

    if (show_ids) try_draw_image_text(view, c[0] + 5, c[1] + 5, pangolin::GlFont::I().Text("%d", lmid));
    if (show_responses && responses.count(lmid) > 0)
      try_draw_image_text(view, c[0] + 5, c[1], pangolin::GlFont::I().Text("%.1f", responses.at(lmid)));
  }

  pangolin::GlFont::I().Text("Detected %d keypoints", kp_map.size()).Draw(5, 40);
}

void VIOUIBase::do_show_highlights(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  pangolin::ImageView& view = *img_view.at(cam_id);

  glColor3ubv(vis::GREEN);

  const std::vector<Keypoints>& keypoints = curr_vis_data->opt_flow_res->keypoints;
  if (cam_id >= keypoints.size()) return;

  const Keypoints& kp_map = keypoints[cam_id];

  for (const auto& [kpid, kpt] : kp_map) {
    if (!is_selected(highlights, kpid)) continue;
    float u = kpt.translation().x();
    float v = kpt.translation().y();
    pangolin::glDrawCirclePerimeter(u, v, 3);
    if (show_ids) try_draw_image_text(view, u, v + 5, pangolin::GlFont::I().Text("%lu", kpid));
  }
}

void VIOUIBase::do_show_tracking_guess(size_t cam_id, size_t frame_id, const VioVisualizationData::Ptr& prev_vis_data) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  if (frame_id < 1) return;

  auto new_kpts = curr_vis_data->opt_flow_res->keypoints[cam_id];
  auto prev_kpts = prev_vis_data->opt_flow_res->keypoints[cam_id];
  auto guess_obs = curr_vis_data->opt_flow_res->tracking_guesses[cam_id];

  std::vector<Vector2f> prev_lines;
  std::vector<Vector2f> prev_points;
  std::vector<Vector2f> guess_lines;
  std::vector<Vector2f> guess_points;
  std::vector<Vector2f> now_points;

  prev_lines.reserve(new_kpts.size());
  prev_points.reserve(new_kpts.size());
  guess_lines.reserve(new_kpts.size());
  guess_points.reserve(new_kpts.size());
  now_points.reserve(new_kpts.size());

  float radius = 3.0F;

  // Draw tracked features in previous frame
  for (auto& [kpid, kpt] : new_kpts) {
    if (prev_kpts.count(kpid) == 0) continue;
    if (guess_obs.count(kpid) == 0) continue;

    bool show = !filter_highlights || is_selected(highlights, kpid);
    if (!show) continue;

    auto n = kpt.translation();
    auto p = prev_kpts.at(kpid).translation();
    auto g = guess_obs.at(kpid).translation();

    now_points.emplace_back(n);

    prev_lines.emplace_back(p);
    prev_lines.emplace_back(n);
    prev_points.emplace_back(p);

    guess_lines.emplace_back(g);
    guess_lines.emplace_back(n);
    guess_points.emplace_back(g);
  }

  glColor4f(1, 0.59, 0, 0.9);
  glDrawCirclePerimeters(now_points, radius);

  glColor4f(0.93, 0.42, 0, 0.3);
  pangolin::glDrawLines(prev_lines);
  glDrawCirclePerimeters(prev_points, radius);

  glColor4f(1, 0.59, 0, 0.5);
  pangolin::glDrawLines(guess_lines);
  glDrawCirclePerimeters(guess_points, radius);
}

void VIOUIBase::do_show_recall_guesses(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  auto new_kpts = curr_vis_data->opt_flow_res->keypoints.at(cam_id);
  auto guess_obs = curr_vis_data->opt_flow_res->recall_guesses.at(cam_id);

  std::vector<Vector2f> guess_lines;
  std::vector<Vector2f> guess_points;

  guess_lines.reserve(new_kpts.size());
  guess_points.reserve(new_kpts.size());

  float radius = 1.0F;
  glColor4f(255 / 255.0F, 13 / 255.0F, 134 / 255.0F, 0.9);

  // Draw tracked features in previous frame
  for (auto& [kpid, kpt] : guess_obs) {
    bool show = !filter_highlights || is_selected(highlights, kpid);
    if (!show) continue;

    auto g = kpt.translation();
    guess_points.emplace_back(g);
    pangolin::GlFont::I().Text("%zu", kpid).Draw(5 + g.x(), 5 + g.y());

    if (new_kpts.count(kpid) > 0) {
      auto n = new_kpts.at(kpid).translation();
      guess_lines.emplace_back(g);
      guess_lines.emplace_back(n);
    }
  }

  pangolin::glDrawLines(guess_lines);
  glDrawCirclePerimeters(guess_points, radius);
}

void VIOUIBase::do_show_tracking_guess_vio(size_t cam_id, size_t frame_id, const VioDatasetPtr& vio_dataset,
                                           const std::unordered_map<int64_t, VioVisualizationData::Ptr>& vis_map) {
  if (frame_id < 1) return;

  int64_t prev_ts = vio_dataset->get_image_timestamps().at(frame_id - 1);
  auto prev_it = vis_map.find(prev_ts);
  if (prev_it == vis_map.end()) return;
  const VioVisualizationData::Ptr& prev_vis_data = prev_it->second;

  do_show_tracking_guess(cam_id, frame_id, prev_vis_data);
}

void VIOUIBase::do_show_matching_guesses(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  auto new_kpts = curr_vis_data->opt_flow_res->keypoints.at(cam_id);
  auto cam0_kpts = curr_vis_data->opt_flow_res->keypoints.at(0);
  auto guess_obs = curr_vis_data->opt_flow_res->matching_guesses.at(cam_id);

  std::vector<Vector2f> cam0_lines;
  std::vector<Vector2f> cam0_points;
  std::vector<Vector2f> guess_lines;
  std::vector<Vector2f> guess_points;
  std::vector<Vector2f> now_points;

  cam0_lines.reserve(new_kpts.size());
  cam0_points.reserve(new_kpts.size());
  guess_lines.reserve(new_kpts.size());
  guess_points.reserve(new_kpts.size());
  now_points.reserve(new_kpts.size());

  float radius = 3.0F;

  // Draw tracked features in previous frame
  for (auto& [kpid, kpt] : new_kpts) {
    if (cam0_kpts.count(kpid) == 0) continue;
    if (guess_obs.count(kpid) == 0) continue;

    bool show = !filter_highlights || is_selected(highlights, kpid);
    if (!show) continue;

    auto n = kpt.translation();
    auto c = cam0_kpts.at(kpid).translation();
    auto g = guess_obs.at(kpid).translation();

    now_points.emplace_back(n);

    cam0_lines.emplace_back(c);
    cam0_lines.emplace_back(n);
    cam0_points.emplace_back(c);

    guess_lines.emplace_back(g);
    guess_lines.emplace_back(n);
    guess_points.emplace_back(g);
  }

  glColor4f(0.12, 0.58, 0.95, 0.9);
  glDrawCirclePerimeters(now_points, radius);

  glColor4f(0, 0.73, 0.83, 0.5);
  pangolin::glDrawLines(cam0_lines);
  glDrawCirclePerimeters(cam0_points, radius);

  glColor4f(0.12, 0.58, 0.95, 0.5);
  pangolin::glDrawLines(guess_lines);
  glDrawCirclePerimeters(guess_points, radius);
}

void VIOUIBase::do_show_masks(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  glColor4f(0.0, 1.0, 1.0, 0.1);
  for (const Rect& m : curr_vis_data->opt_flow_res->input_images->masks[cam_id].masks) {
    pangolin::glDrawRect(m.x, m.y, m.x + m.w, m.y + m.h);
  }
}

void VIOUIBase::do_show_cam0_proj(size_t cam_id, double depth_guess) {
  int C = config.optical_flow_detection_grid_size;

  int w = calib.resolution.at(0).x();
  int x_start = (w % C) / 2;
  int x_stop = x_start + C * (w / C - 1);
  int x_first = x_start + C / 2;
  int x_last = x_stop + C / 2;

  int h = calib.resolution.at(0).y();
  int y_start = (h % C) / 2;
  int y_stop = y_start + C * (h / C - 1);
  int y_first = y_start + C / 2;
  int y_last = y_stop + C / 2;

  std::vector<Vector2d> points;
  auto drawPoint = [&points, this, w, h, depth_guess](float u, float v, int j, bool draw_c0_uv) {
    Vector2d ci_uv{u, v};
    Vector2d c0_uv;
    double _;
    bool projected = calib.projectBetweenCams(ci_uv, depth_guess, c0_uv, _, j, 0);
    bool in_bounds = c0_uv.x() >= 0 && c0_uv.x() < w && c0_uv.y() >= 0 && c0_uv.y() < h;
    bool valid = projected && in_bounds;

    // Define color
    GLfloat invalid_color[4] = {1, 0, 0, 0.5};      // red
    GLfloat in_bounds_color[4] = {1, 0.5, 0, 0.5};  // orange
    GLfloat projected_color[4] = {1, 0.9, 0, 0.5};  // yellow
    GLfloat valid_color[4] = {0, 1, 0, 0.5};        // green
    GLfloat* color = invalid_color;
    if (valid) {
      color = valid_color;
    } else if (projected) {
      color = projected_color;
    } else if (in_bounds) {
      color = in_bounds_color;
    }
    glColor4fv(color);

    // Press L key twice in viewer to be able to see out-of-bounds points
    if (projected) {
      points.push_back(c0_uv);
    }

    if (draw_c0_uv) {
      pangolin::glDrawCircle(c0_uv.x(), c0_uv.y(), 2);
    } else {
      pangolin::glDrawCircle(ci_uv.x(), ci_uv.y(), 2);
    }
  };

  if (cam_id == 0) {
    size_t num_cams = calib.resolution.size();
    for (size_t target_cam = 1; target_cam < num_cams; target_cam++) {
#if 1  // Draw perimeter of projected-to-cam0 grid
      int x = x_first;
      int y = y_first;
      for (; x <= x_last; x += C) drawPoint(x, y, target_cam, true);
      for (x = x_last; y <= y_last; y += C) drawPoint(x, y, target_cam, true);
      for (y = y_last; x >= x_first; x -= C) drawPoint(x, y, target_cam, true);
      for (x = x_first; y >= y_first; y -= C) drawPoint(x, y, target_cam, true);

#else  // Draw full projected-to-cam0 grid
      for (int y = x_first; y <= y_last; y += C) {
        for (int x = y_first; x <= x_last; x += C) {
          drawPoint(x, y, target_cam, true);
        }
      }
#endif

      glColor4f(0.0, 1.0, 0.0, 0.5);
      pangolin::glDrawLineLoop(points);
    }
  } else {
    for (int y = y_first; y < h; y += C) {
      for (int x = x_first; x < w; x += C) {
        drawPoint(x, y, cam_id, false);
      }
    }
  }
}

void VIOUIBase::do_show_grid() {
  glColor4f(1.0, 0.0, 1.0, 0.25);

  int C = config.optical_flow_detection_grid_size;

  int w = calib.resolution.at(0).x();
  int x_start = (w % C) / 2;
  int x_stop = x_start + C * (w / C - 1);
  int x_end = x_stop + C;

  int h = calib.resolution.at(0).y();
  int y_start = (h % C) / 2;
  int y_stop = y_start + C * (h / C - 1);
  int y_end = y_stop + C;

  std::vector<Vector2f> grid_lines;
  for (int x = x_start; x <= x_end; x += C) {
    grid_lines.emplace_back(x, y_start);
    grid_lines.emplace_back(x, y_end);
  }
  for (int y = y_start; y <= y_end; y += C) {
    grid_lines.emplace_back(x_start, y);
    grid_lines.emplace_back(x_end, y);
  }
  pangolin::glDrawLines(grid_lines);
}

void VIOUIBase::do_show_safe_radius() {
  if (config.optical_flow_image_safe_radius == 0) return;
  glColor4f(1.0, 0.0, 1.0, 0.25);
  int w = calib.resolution.at(0).x();
  int h = calib.resolution.at(0).y();
  pangolin::glDrawCirclePerimeter(w / 2, h / 2, config.optical_flow_image_safe_radius);
}

void VIOUIBase::do_show_guesses(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  if (cam_id == 0) return;

  const auto keypoints0 = curr_vis_data->projections->at(0);
  const auto keypoints1 = curr_vis_data->projections->at(cam_id);

  double avg_invdepth = 0;
  double num_features = 0;
  for (const auto& cam_projs : *curr_vis_data->projections) {
    for (const Vector4d& v : cam_projs) avg_invdepth += v.z();
    num_features += cam_projs.size();
  }
  bool valid = avg_invdepth > 0 && num_features > 0;
  float default_depth = config.optical_flow_matching_default_depth;
  double avg_depth = valid ? num_features / avg_invdepth : default_depth;

  for (const Vector4d& kp1 : keypoints1) {
    double u1 = kp1.x();
    double v1 = kp1.y();
    // double invdist1 = kp1.z();
    double id1 = kp1.w();

    bool show = !filter_highlights || is_selected(highlights, id1);
    if (!show) continue;

    double u0 = 0;
    double v0 = 0;
    bool found = false;
    for (const Vector4d& kp0 : keypoints0) {  // Find match in keypoints0
      double id0 = kp0.w();
      if (id1 != id0) continue;
      u0 = kp0.x();
      v0 = kp0.y();
      found = true;
      break;
    }

    // Display guess error if this is a stereo feature
    // NOTE: keep in mind that these guesses are not really the guesses
    // used to detect the feature, but the guess we would use if we were
    // to detect the feature right now.
    if (found) {
      // Guess if we were using SAME_PIXEL
      if (show_same_pixel_guess) {
        glColor3f(0, 1, 1);  // Cyan
        pangolin::glDrawLine(u1, v1, u0, v0);
      }

      // Guess if we were using REPROJ_FIX_DEPTH
      if (show_reproj_fix_depth_guess) {
        glColor3f(1, 1, 0);  // Yellow
        auto off = calib.viewOffset({u0, v0}, fixed_depth, 0, cam_id);
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }

      // Guess if we were using REPROJ_AVG_DEPTH
      if (show_reproj_avg_depth_guess) {
        glColor3f(1, 0, 1);  // Magenta
        auto off = calib.viewOffset({u0, v0}, avg_depth, 0, cam_id);
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }

      // Guess with the current guess type
      if (show_active_guess) {
        glColor3f(1, 0, 0);  // Red
        Vector2d off{0, 0};
        if (config.optical_flow_matching_guess_type != MatchingGuessType::SAME_PIXEL) {
          off = calib.viewOffset({u0, v0}, curr_vis_data->opt_flow_res->input_images->depth_guess, 0, cam_id);
        }
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }
    }
  }
}

void VIOUIBase::do_show_obs(size_t cam_id) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  pangolin::ImageView& view = *img_view.at(cam_id);

  glLineWidth(1.0);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  size_t num_cams = calib.resolution.size();
  if (cam_id < num_cams) {
    const auto& points = curr_vis_data->projections->at(cam_id);

    if (!points.empty()) {
      double min_id = points[0][2];
      double max_id = points[0][2];

      for (const auto& points2 : *curr_vis_data->projections) {
        for (const auto& p : points2) {
          min_id = std::min(min_id, p[2]);
          max_id = std::max(max_id, p[2]);
        }
      }

      for (const auto& c : points) {
        double u = c[0];
        double v = c[1];
        double depth = c[2] != 0.0 ? 1.0 / c[2] : std::numeric_limits<double>::infinity();
        int id = c[3];

        bool show = !filter_highlights || is_selected(highlights, id);
        if (!show) continue;

        double width = calib.resolution.at(0).x();
        double unit_radius = width / 96;
        double radius = unit_radius / depth;

        double min_depth = 1.0 / 3;  // 1/3 comes from how valid_kp is computed in sqrt_keypoint_vio.cpp
        double max_depth = 20;       // And this is arbitrary
        double max_radius = unit_radius / min_depth;
        double min_radius = unit_radius * min_depth;

        bool clamped = depth < min_depth || depth > max_depth;
        double cradius = std::clamp(radius, min_radius, max_radius);

        float t = (cradius - min_radius) / (max_radius - min_radius);
        auto [r, g, b] = color_lerp(t);

        if (clamped) {  // Mark clamped points in UI
          glColor4f(r, g, b, 0.15);
          pangolin::glDrawCircle(u, v, cradius);
          glColor4f(r, g, b, 1);
        } else {
          glColor4f(r, g, b, 1);
          pangolin::glDrawCirclePerimeter(u, v, cradius);
        }

        if (show_ids) try_draw_image_text(view, u, v, pangolin::GlFont::I().Text("%d", id));
        if (show_depth) pangolin::GlFont::I().Text("%.3lf m", depth).Draw(u, v + 5);
      }
    }

    if (show_guesses) do_show_guesses(cam_id);

    glColor3f(0.0, 1.0, 0.0);
    pangolin::GlFont::I().Text("Tracked %d points", points.size()).Draw(5, 20);
  }
}

void VIOUIBase::draw_blocks_overlay(pangolin::ImageView& blocks_view) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  UIMAT m = (UIMAT)mat_to_show.Get();
  if (is_jacobian(m)) {
    UIJacobians& uijac = curr_vis_data->getj(m);
    draw_jacobian_overlay(blocks_view, uijac);
  } else if (is_hessian(m)) {
    UIHessians& uihes = curr_vis_data->geth(m);
    draw_hessian_overlay(blocks_view, uihes);
  } else {
    BASALT_ASSERT(false);
  }
}

void VIOUIBase::draw_jacobian_overlay(pangolin::ImageView& blocks_view, const UIJacobians& uij) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  const auto& uibs = filter_highlights ? uij.Jr_h : uij.Jr;
  if (!uibs) return;

  const std::vector<UILandmarkBlock>& lmbs = uibs->blocks;
  if (lmbs.empty()) return;

  size_t pad = show_ids ? 12 : 0;
  size_t w = uibs->getW();
  size_t h = uibs->getH();
  size_t W = w + pad;
  size_t H = h + pad;
  size_t side = max(W, H);
  long xoffh = int((side - W) / 2);  // Offset to center view
  long xoff = xoffh + pad;           // Offset plus padding for ids
  long yoff = pad;                   // Offset plus padding for ids
  const auto& aom = uibs->aom.abs_order_map;

  // Draw column separators
  glLineWidth(0.25);
  glColor3ubv(BLUE);
  pangolin::glDrawLine(xoff - 0.5, -0.5, xoff - 0.5, H - 0.5);  // Matrix start
  for (const auto& [ts, idx_size] : aom) {                      // Keyframe/frame end
    const auto [idx, size] = idx_size;
    pangolin::glDrawLine(xoff + idx + size - 0.5, -0.5, xoff + idx + size - 0.5, H - 0.5);
    if (show_ids) {
      bool keyframed = curr_vis_data->keyframed_idx.count(ts) > 0;
      bool marginalized = curr_vis_data->marginalized_idx.count(ts) > 0;
      bool present = curr_vis_data->frame_idx.count(ts) > 0;
      if (!keyframed && !marginalized && !present) BASALT_ASSERT(false);

      size_t fid = (keyframed      ? curr_vis_data->keyframed_idx[ts]
                    : marginalized ? curr_vis_data->marginalized_idx[ts]
                                   : curr_vis_data->frame_idx[ts]);
      glColor3ubv(keyframed ? GREEN : marginalized ? RED : BLUE);
      auto text = pangolin::GlFont::I().Text("%lu", fid);
      try_draw_image_text(blocks_view, xoff + idx, pad / 2, text);
      glColor3ubv(BLUE);
    }
  }
  pangolin::glDrawLine(xoff + w - 4 - 0.5, -0.5, xoff + w - 4 - 0.5, H - 0.5);  // Landmark start
  pangolin::glDrawLine(xoff + w - 1 - 0.5, -0.5, xoff + w - 1 - 0.5, H - 0.5);  // Residual start
  pangolin::glDrawLine(xoff + w - 0 - 0.5, -0.5, xoff + w - 0 - 0.5, H - 0.5);  // Matrix end

  // Draw row separators
  size_t i = 0;
  for (const UILandmarkBlock& b : lmbs) {
    bool highlighted = show_highlights && is_selected(highlights, b.lmid);
    glColor3ubv(highlighted ? GREEN : BLUE);
    pangolin::glDrawLine(xoffh - 0.5, yoff + i - 0.5, xoffh + W - 0.5, yoff + i - 0.5);

    if (show_ids) {
      auto text = pangolin::GlFont::I().Text("%lu", b.lmid);
      try_draw_image_text(blocks_view, xoffh + 2, yoff + i + b.storage->rows() / 2.0F, text);
    }

    if (show_block_vals) {  // Draw cell values
      for (long y = 0; y < b.storage->rows(); y++) {
        for (long x = 0; x < b.storage->cols(); x++) {
          float c = b.storage->coeff(y, x);
          if (c == 0) continue;

          float u = x + xoff - 0.25;
          float v = i + y + yoff;
          glColor3ubv(c > 0 ? GREEN : RED);
          auto text = SMALL_FONT.Text("%.2f", abs(c));
          try_draw_image_text(blocks_view, u, v, text);
        }
      }
    }

    i += b.storage->rows();
  }
}

void VIOUIBase::draw_hessian_overlay(pangolin::ImageView& blocks_view, const UIHessians& uihes) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  if (uihes.H == nullptr || uihes.b == nullptr || uihes.aom == nullptr) return;

  const auto H = uihes.H;
  const auto b = uihes.b;
  const auto aom = uihes.aom;

  size_t pad = show_ids ? 6 : 0;
  size_t w = H->cols() + 1;
  size_t h = H->rows();
  size_t pw = w + pad;
  size_t ph = h + pad;
  size_t side = max(pw, ph);
  long xoffh = int((side - pw) / 2);  // Offset to center view
  long xoff = xoffh + pad;            // Offset plus padding for ids
  long yoff = pad;                    // Offset plus padding for ids

  // Draw column and row separators
  glLineWidth(0.25);
  glColor3ubv(BLUE);
  pangolin::glDrawLine(xoff - 0.5, -0.5, xoff - 0.5, ph - 0.5);               // Matrix start col
  pangolin::glDrawLine(xoffh - 0.5, yoff - 0.5, xoff + w - 0.5, yoff - 0.5);  // Matrix start row
  for (const auto& [ts, idx_size] : aom->abs_order_map) {                     // Keyframe/frame end
    const auto [idx, size] = idx_size;
    float p = xoff + idx + size - 0.5;
    pangolin::glDrawLine(p, -0.5, p, ph - 0.5);               // Column
    pangolin::glDrawLine(xoffh - 0.5, p, xoff + w - 0.5, p);  // Row

    if (show_ids) {
      bool keyframed = curr_vis_data->keyframed_idx.count(ts) > 0;
      bool marginalized = curr_vis_data->marginalized_idx.count(ts) > 0;
      bool present = curr_vis_data->frame_idx.count(ts) > 0;
      if (!keyframed && !marginalized && !present) BASALT_ASSERT(false);

      size_t fid = (keyframed      ? curr_vis_data->keyframed_idx[ts]
                    : marginalized ? curr_vis_data->marginalized_idx[ts]
                                   : curr_vis_data->frame_idx[ts]);
      glColor3ubv(keyframed ? GREEN : marginalized ? RED : BLUE);
      auto text = pangolin::GlFont::I().Text("%lu", fid);
      try_draw_image_text(blocks_view, xoff + idx, pad / 2, text);
      try_draw_image_text(blocks_view, 0, yoff + idx + pad / 2, text);
      glColor3ubv(BLUE);
    }
  }
  pangolin::glDrawLine(xoff + w - 1 - 0.5, -0.5, xoff + w - 1 - 0.5, ph - 0.5);  // Column for b

  if (show_block_vals) {  // Draw cell values
    for (long y = 0; y < H->rows(); y++) {
      for (long x = 0; x < H->cols() + 1; x++) {
        float c = x != H->cols() ? H->coeff(y, x) : b->coeff(y);
        if (c == 0) continue;
        glColor3ubv(c > 0 ? GREEN : RED);
        auto text = SMALL_FONT.Text("%.2f", abs(c));
        try_draw_image_text(blocks_view, x + xoff - 0.25, yoff + y, text);
      }
    }
  }
}

bool VIOUIBase::do_toggle_blocks(pangolin::View* blocks_display, pangolin::View* plot_display,
                                 pangolin::View* img_view_display, pangolin::Attach UI_WIDTH) {
  blocks_display->ToggleShow();
  bool show_blocks = blocks_display->IsShown();

  size_t child_count = img_view_display->NumVisibleChildren();
  if (child_count == 0) return show_blocks;

  if (!show_blocks) {
    plot_display->SetBounds(0.0, 0.4, UI_WIDTH, 1.0);
    return show_blocks;
  }

  pangolin::ImageView* last_img_view = (pangolin::ImageView*)&img_view_display->VisibleChild(child_count - 1);
  pangolin::XYRangef range = last_img_view->GetDefaultView();
  float xmax = -1;
  float ymax = -1;
  last_img_view->ImageToScreen(last_img_view->v, range.x.max, range.y.max, xmax, ymax);

  auto X_ATTACH = pangolin::Attach::Pix(xmax);
  auto Y_ATTACH = pangolin::Attach::Pix(ymax);
  blocks_display->SetBounds(0.0, Y_ATTACH, UI_WIDTH, X_ATTACH);
  plot_display->SetBounds(0.0, 0.4, X_ATTACH, 1.0);

  return show_blocks;
}

void VIOUIBase::do_show_blocks(const shared_ptr<ImageView>& blocks_view) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  UIMAT m = (UIMAT)mat_to_show.Get();
  if (is_jacobian(m)) {
    UIJacobians& uijac = curr_vis_data->getj(m);
    do_show_jacobian(blocks_view, uijac);
  } else if (is_hessian(m)) {
    UIHessians& uihes = curr_vis_data->geth(m);
    do_show_hessian(blocks_view, uihes);
  } else {
    BASALT_ASSERT(false);
  }
}

void VIOUIBase::do_show_hessian(const shared_ptr<ImageView>& blocks_view, UIHessians& uih) {
  if (uih.H == nullptr || uih.b == nullptr) return;

  const auto H = uih.H;
  const auto b = uih.b;

  std::shared_ptr<ManagedImage<uint8_t>> mat;
  size_t w = H->cols() + 1;
  size_t h = H->rows();
  size_t pad = show_ids ? 6 : 0;
  size_t pw = w + pad;
  size_t ph = h + pad;

  size_t side = max(pw, ph);
  mat = std::make_shared<ManagedImage<uint8_t>>(side, side);
  mat->Memset(0);
  long xoffh = int((side - pw) / 2);  // Offset to center view
  long xoff = xoffh + pad;            // Offset plus padding for ids
  long yoff = pad;                    // Offset plus padding for ids

  float min = MAXFLOAT;
  float max = -MAXFLOAT;
  for (long y = 0; y < H->rows(); y++) {
    for (long x = 0; x < H->cols() + 1; x++) {
      float v = std::abs(x != H->cols() ? H->coeff(y, x) : b->coeff(y));
      if (v < min) min = v;
      if (v > max) max = v;
    }
  }

  max = std::min(max, 255000.0F);

  const uint8_t max_pixel = 245;
  for (long y = 0; y < H->rows(); y++) {
    for (long x = 0; x < H->cols() + 1; x++) {
      float value = std::abs(x != H->cols() ? H->coeff(y, x) : b->coeff(y));
      value = std::min(value, max);
      bool is_not_zero = value > 0;
      uint8_t pixel = (1 - value / max) * 255.0F;
      if (pixel > max_pixel && is_not_zero) pixel = max_pixel;
      (*mat)(x + xoff, yoff + y) = pixel;
    }
  }

  // Fill in row and height title borders
  uint8_t fill_color = 230;
  for (size_t i = 0; i < pad; i++)
    for (size_t j = 0; j < w; j++) (*mat)(xoff + j, i) = fill_color;

  for (size_t i = 0; i < ph; i++)
    for (size_t j = 0; j < pad; j++) (*mat)(xoffh + j, i) = fill_color;

  uih.img = mat;

  pangolin::GlPixFormat fmt;
  fmt.glformat = GL_LUMINANCE;
  fmt.gltype = GL_UNSIGNED_BYTE;
  fmt.scalable_internal_format = GL_LUMINANCE8;
  blocks_view->SetImage(mat->ptr, mat->w, mat->h, mat->pitch, fmt);
}

void VIOUIBase::do_show_jacobian(const shared_ptr<ImageView>& blocks_view, UIJacobians& uij) {
  UILandmarkBlocks::Ptr uibs = uij.Jr;
  std::shared_ptr<ManagedImage<uint8_t>> mat;

  if (!uibs || (uibs && uibs->blocks.empty())) {
    mat = std::make_shared<ManagedImage<uint8_t>>(1, 1);
    mat->Memset(127);
  } else if (uij.img != nullptr) {  // Reuse same image
    mat = uij.img;
  } else {
    // Use a different UILandmarkBlocks when landmarks are highlighted
    UILandmarkBlocks::Ptr hluibs;
    if (filter_highlights) {
      hluibs = std::make_shared<UILandmarkBlocks>();
      hluibs->aom = uibs->aom;
      for (const UILandmarkBlock& b : uibs->blocks)
        if (is_selected(highlights, b.lmid)) hluibs->blocks.push_back(b);
      uij.Jr_h = hluibs;
    }

    const UILandmarkBlocks::Ptr u = filter_highlights ? hluibs : uibs;
    const std::vector<UILandmarkBlock>& lmbs = u->blocks;
    size_t pad = show_ids ? 12 : 0;
    size_t w = u->getW();
    size_t h = u->getH();
    size_t W = w + pad;
    size_t H = h + pad;
    size_t side = max(W, H);
    mat = std::make_shared<ManagedImage<uint8_t>>(side, side);
    mat->Memset(0);
    long xoffh = int((side - W) / 2);  // Offset to center view
    long xoff = xoffh + pad;           // Offset plus padding for ids
    long yoff = pad;                   // Offset plus padding for ids

    size_t i = 0;
    float min = MAXFLOAT;
    float max = -MAXFLOAT;
    for (const UILandmarkBlock& b : lmbs) {
      for (long y = 0; y < b.storage->rows(); y++) {
        for (long x = 0; x < b.storage->cols(); x++) {
          float v = std::abs(b.storage->coeff(y, x));
          if (v < min) min = v;
          if (v > max) max = v;
        }
      }
      i += b.storage->rows();
    }

    i = 0;
    const uint8_t max_pixel = 245;
    for (const UILandmarkBlock& b : lmbs) {
      for (long y = 0; y < b.storage->rows(); y++) {
        for (long x = 0; x < b.storage->cols(); x++) {
          float value = std::abs(b.storage->coeff(y, x));
          bool is_not_zero = value > 0;
          uint8_t pixel = (1 - value / max) * 255.0F;
          if (pixel > max_pixel && is_not_zero) pixel = max_pixel;
          (*mat)(x + xoff, i + yoff + y) = pixel;
        }
      }
      i += b.storage->rows();
    }

    // Fill in row and height title borders
    uint8_t fill_color = 230;
    for (size_t i = 0; i < pad; i++)
      for (size_t j = 0; j < u->getW(); j++) (*mat)(xoff + j, i) = fill_color;

    for (size_t i = 0; i < H; i++)
      for (size_t j = 0; j < pad; j++) (*mat)(xoffh + j, i) = fill_color;

    uij.img = mat;
  }

  pangolin::GlPixFormat fmt;
  fmt.glformat = GL_LUMINANCE;
  fmt.gltype = GL_UNSIGNED_BYTE;
  fmt.scalable_internal_format = GL_LUMINANCE8;
  blocks_view->SetImage(mat->ptr, mat->w, mat->h, mat->pitch, fmt);
}

//! Parse a set of numbers described in @p str. Example inputs: "1,3,5-10", "1000-2000,3,5-7"
Selection parse_selection(const string& str) {
  Selection nodes{};

  try {
    std::istringstream token_stream(str);
    vector<string> tokens;
    for (string token; std::getline(token_stream, token, ',');) tokens.push_back(token);

    for (const string& token : tokens) {
      size_t dash_pos = token.find('-');
      bool is_range = dash_pos != string::npos;
      size_t a = stoull(token.substr(0, dash_pos));
      size_t b = is_range ? stoull(token.substr(dash_pos + 1)) : -1;
      nodes.push_back({is_range, a, b});
    }
  } catch (...) {
    std::cout << "Invalid selection string: " << str << std::endl;
  }

  return nodes;
}

bool is_selected(const Selection& selection, size_t n) {
  for (const SelectionNode& node : selection)
    if (node.contains(n)) return true;
  return false;
}

bool VIOUIBase::do_follow_highlight(bool smooth_zoom) {
  const VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return false;

  for (size_t cam_id = 0; cam_id < img_view.size(); cam_id++) {
    std::shared_ptr<pangolin::ImageView> v = img_view.at(cam_id);
    if (v == nullptr) continue;

    if (highlights.size() != 1 || highlights[0].is_range) {
      v->ResetView();
      continue;
    };

    const std::vector<Keypoints>& keypoints = curr_vis_data->opt_flow_res->keypoints;
    if (cam_id >= keypoints.size()) {
      v->ResetView();
      continue;
    };

    const Keypoints& camkps = keypoints[cam_id];
    size_t kpid = highlights[0].a;
    if (camkps.count(kpid) == 0) {
      v->ResetView();
      continue;
    };

    Vector2f kp = camkps.at(kpid).translation();

    pangolin::XYRangef full_range = v->GetDefaultView();
    const float w = full_range.x.AbsSize();
    const float h = full_range.y.AbsSize();

    const float tw = 32;
    const float th = (tw / w) * h;
    float l = kp.x() - tw / 2;
    float r = kp.x() + tw / 2;
    float t = kp.y() - th / 2;
    float b = kp.y() + th / 2;
    pangolin::XYRangef zoomed_range = {{l, r}, {t, b}};

    if (smooth_zoom)
      v->SetViewSmooth(zoomed_range);
    else
      v->SetView(zoomed_range);
  }
  return true;
}

void VIOUIBase::do_render_camera(const Sophus::SE3d& T_w_c, size_t i, size_t ts, const uint8_t* color) {
  VioVisualizationData::Ptr curr_vis_data = get_curr_vis_data();
  if (curr_vis_data == nullptr) return;

  bool keyframed = curr_vis_data->keyframed_idx.count(ts) > 0;
  bool marginalized = curr_vis_data->marginalized_idx.count(ts) > 0;
  bool present = curr_vis_data->frame_idx.count(ts) > 0;
  if (!keyframed && !marginalized && !present) BASALT_ASSERT(false);

  size_t fid = (keyframed      ? curr_vis_data->keyframed_idx[ts]
                : marginalized ? curr_vis_data->marginalized_idx[ts]
                               : curr_vis_data->frame_idx[ts]);
  const uint8_t* fid_color = keyframed ? GREEN : marginalized ? RED : BLUE;
  render_camera(T_w_c.matrix(), 2.0F, color, 0.1F, show_ids && i == 0, fid, fid_color);
}

}  // namespace basalt::vis
