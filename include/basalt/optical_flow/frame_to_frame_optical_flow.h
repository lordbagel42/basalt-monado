/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
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

#pragma once

#include <memory>

#include <sophus/se2.hpp>
#include "basalt/imu/imu_types.h"
#include "basalt/imu/preintegration.h"
#include "basalt/utils/common_types.h"
#include "basalt/utils/imu_types.h"
#include "sophus/se3.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/optical_flow/patch.h>

#include <basalt/utils/keypoints.h>
#include <basalt/vi_estimator/landmark_database.h>

namespace basalt {

/// Unlike PatchOpticalFlow, FrameToFrameOpticalFlow always tracks patches
/// against the previous frame, not the initial frame where a track was created.
/// While it might cause more drift of the patch location, it leads to longer
/// tracks in practice.
template <typename Scalar, template <typename> typename Pattern>
class FrameToFrameOpticalFlow : public OpticalFlowTyped<Scalar, Pattern> {
 public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;

  using typename OpticalFlowTyped<Scalar, Pattern>::PatchT;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector2;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix2;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector3;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix3;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector4;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix4;
  using typename OpticalFlowTyped<Scalar, Pattern>::SE2;
  using typename OpticalFlowTyped<Scalar, Pattern>::SE3;
  using OpticalFlowTyped<Scalar, Pattern>::getNumCams;
  using OpticalFlowTyped<Scalar, Pattern>::calib;
  using OpticalFlowTyped<Scalar, Pattern>::E;

  using OpticalFlowBase::config;
  using OpticalFlowBase::depth_guess;
  using OpticalFlowBase::first_state_arrived;
  using OpticalFlowBase::frame_counter;
  using OpticalFlowBase::input_depth_queue;
  using OpticalFlowBase::input_img_queue;
  using OpticalFlowBase::input_imu_queue;
  using OpticalFlowBase::input_lm_bundle_queue;
  using OpticalFlowBase::input_state_queue;
  using OpticalFlowBase::last_keypoint_id;
  using OpticalFlowBase::latest_lm_bundle;
  using OpticalFlowBase::latest_state;
  using OpticalFlowBase::old_pyramid;
  using OpticalFlowBase::output_queue;
  using OpticalFlowBase::patch_coord;
  using OpticalFlowBase::predicted_state;
  using OpticalFlowBase::processing_thread;
  using OpticalFlowBase::pyramid;
  using OpticalFlowBase::show_gui;
  using OpticalFlowBase::t_ns;
  using OpticalFlowBase::transforms;

  FrameToFrameOpticalFlow(const VioConfig& conf, const Calibration<double>& cal)
      : OpticalFlowTyped<Scalar, Pattern>(conf, cal),
        accel_cov(cal.dicrete_time_accel_noise_std().array().square()),
        gyro_cov(cal.dicrete_time_gyro_noise_std().array().square()) {
    latest_state = std::make_shared<PoseVelBiasState<double>>();
    predicted_state = std::make_shared<PoseVelBiasState<double>>();
    patches.reserve(3000);
  }

  void processingLoop() override {
    using std::make_shared;
    OpticalFlowInput::Ptr img;

    while (true) {
      input_img_queue.pop(img);

      if (img == nullptr) {
        if (output_queue) output_queue->push(nullptr);
        break;
      }
      img->addTime("frames_received");

      while (input_depth_queue.try_pop(depth_guess)) continue;
      if (show_gui) img->depth_guess = depth_guess;

      while (input_lm_bundle_queue.try_pop(latest_lm_bundle)) continue;

      if (!input_state_queue.empty()) {
        while (input_state_queue.try_pop(latest_state)) continue;  // Flush
        first_state_arrived = true;
      } else if (first_state_arrived) {
        latest_state = make_shared<PoseVelBiasState<double>>(*predicted_state);
      }

      if (first_state_arrived) {
        auto pim = processImu(img->t_ns);
        pim.predictState(*latest_state, constants::g, *predicted_state);
      }

      processFrame(img->t_ns, img);
    }
  }

  IntegratedImuMeasurement<double> processImu(int64_t curr_t_ns) {
    int64_t prev_t_ns = t_ns;
    Vector3d bg = latest_state->bias_gyro;
    Vector3d ba = latest_state->bias_accel;
    IntegratedImuMeasurement<double> pim{prev_t_ns, bg, ba};

    if (input_imu_queue.empty()) return pim;

    auto pop_imu = [&](ImuData<double>::Ptr& data) -> bool {
      ImuData<double>::Ptr sample;
      input_imu_queue.pop(sample);  // Blocking pop
      if (sample == nullptr) return false;

      // Calibrate sample
      int64_t t = sample->t_ns;
      Vector3 a = calib.calib_accel_bias.getCalibrated(sample->accel.cast<Scalar>());
      Vector3 g = calib.calib_gyro_bias.getCalibrated(sample->gyro.cast<Scalar>());

      data = std::make_shared<ImuData<double>>();
      data->t_ns = t;
      data->accel = a.template cast<double>();
      data->gyro = g.template cast<double>();
      return true;
    };

    typename ImuData<double>::Ptr data = nullptr;
    if (!pop_imu(data)) return pim;

    while (data->t_ns <= prev_t_ns) {
      if (!pop_imu(data)) return pim;
    }

    while (data->t_ns <= curr_t_ns) {
      pim.integrate(*data, accel_cov, gyro_cov);
      if (!pop_imu(data)) return pim;
    }

    // Pretend last IMU sample before "now" happened now
    if (pim.get_start_t_ns() + pim.get_dt_ns() < curr_t_ns) {
      data->t_ns = curr_t_ns;
      pim.integrate(*data, accel_cov, gyro_cov);
    }

    return pim;
  }

  void processFrame(int64_t curr_t_ns, OpticalFlowInput::Ptr& new_img_vec) {
    for (const auto& v : new_img_vec->img_data) {
      if (!v.img.get()) return;
    }

    const size_t num_cams = getNumCams();

    if (t_ns < 0) {
      t_ns = curr_t_ns;

      transforms.reset(new OpticalFlowResult);
      transforms->keypoints.resize(num_cams);
      transforms->tracking_guesses.resize(num_cams);
      transforms->matching_guesses.resize(num_cams);
      transforms->recall_guesses.resize(num_cams);
      transforms->t_ns = t_ns;

      pyramid.reset(new std::vector<ManagedImagePyr<uint16_t>>);
      pyramid->resize(num_cams);

      tbb::parallel_for(tbb::blocked_range<size_t>(0, num_cams), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          pyramid->at(i).setFromImage(*new_img_vec->img_data[i].img, config.optical_flow_levels);
        }
      });

      transforms->input_images = new_img_vec;

      addPoints();
      filterPoints();
    } else {
      t_ns = curr_t_ns;

      old_pyramid = pyramid;

      pyramid.reset(new std::vector<ManagedImagePyr<uint16_t>>);
      pyramid->resize(num_cams);
      tbb::parallel_for(tbb::blocked_range<size_t>(0, num_cams), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          pyramid->at(i).setFromImage(*new_img_vec->img_data[i].img, config.optical_flow_levels);
        }
      });

      OpticalFlowResult::Ptr new_transforms;
      new_transforms.reset(new OpticalFlowResult);
      new_transforms->keypoints.resize(num_cams);
      new_transforms->tracking_guesses.resize(num_cams);
      new_transforms->matching_guesses.resize(num_cams);
      new_transforms->recall_guesses.resize(num_cams);
      new_transforms->t_ns = t_ns;

      SE3 T_i1 = latest_state->T_w_i.template cast<Scalar>();
      SE3 T_i2 = predicted_state->T_w_i.template cast<Scalar>();
      for (size_t i = 0; i < num_cams; i++) {
        SE3 T_c1 = T_i1 * calib.T_i_c[i];
        SE3 T_c2 = T_i2 * calib.T_i_c[i];
        SE3 T_c1_c2 = T_c1.inverse() * T_c2;
        trackPoints(old_pyramid->at(i), pyramid->at(i),  //
                    transforms->keypoints[i], new_transforms->keypoints[i],
                    new_transforms->tracking_guesses[i],  //
                    new_img_vec->masks.at(i), new_img_vec->masks.at(i), T_c1_c2, i, i);
      }

      transforms = new_transforms;
      transforms->input_images = new_img_vec;

      if (config.optical_flow_enable_recall) recallPoints();
      addPoints();
      filterPoints();
    }

    if (output_queue && frame_counter % config.optical_flow_skip_frames == 0) {
      transforms->input_images->addTime("opticalflow_produced");
      output_queue->push(transforms);
    }

    frame_counter++;
  }

  void trackPoints(const ManagedImagePyr<uint16_t>& pyr_1, const ManagedImagePyr<uint16_t>& pyr_2,  //
                   const Keypoints& keypoint_map_1, Keypoints& keypoint_map_2, Keypoints& guesses,  //
                   const Masks& masks1, const Masks& masks2, const SE3& T_c1_c2, size_t cam1, size_t cam2) const {
    size_t num_points = keypoint_map_1.size();

    std::vector<KeypointId> ids;
    Eigen::aligned_vector<Keypoint> init_vec;

    ids.reserve(num_points);
    init_vec.reserve(num_points);

    for (const auto& [kpid, affine] : keypoint_map_1) {
      ids.push_back(kpid);
      init_vec.push_back(affine);
    }

    tbb::concurrent_unordered_map<KeypointId, Keypoint, std::hash<KeypointId>> result, guesses_tbb;

    bool tracking = cam1 == cam2;
    bool matching = cam1 != cam2;
    MatchingGuessType guess_type = config.optical_flow_matching_guess_type;
    bool match_guess_uses_depth = guess_type != MatchingGuessType::SAME_PIXEL;
    const bool use_depth = tracking || (matching && match_guess_uses_depth);
    const double depth = depth_guess;

    auto compute_func = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const KeypointId id = ids[r];

        const Eigen::AffineCompact2f& transform_1 = init_vec[r];
        Eigen::AffineCompact2f transform_2 = transform_1;

        auto t1 = transform_1.translation();
        auto t2 = transform_2.translation();

        if (masks1.inBounds(t1.x(), t1.y())) continue;

        bool valid = true;

        Eigen::Vector2f off{0, 0};

        if (use_depth) {
          Vector2 t2_guess;
          Scalar _;
          calib.projectBetweenCams(t1, depth, t2_guess, _, T_c1_c2, cam1, cam2);
          off = t2 - t2_guess;
        }

        t2 -= off;  // This modifies transform_2

        if (show_gui) {
          guesses_tbb[id] = transform_2;
        }

        valid = t2(0) >= 0 && t2(1) >= 0 && t2(0) < pyr_2.lvl(0).w && t2(1) < pyr_2.lvl(0).h;
        if (!valid) continue;

        valid = trackPoint(pyr_1, pyr_2, transform_1, transform_2);
        if (!valid) continue;

        if (masks2.inBounds(t2.x(), t2.y())) continue;

        Eigen::AffineCompact2f transform_1_recovered = transform_2;
        auto t1_recovered = transform_1_recovered.translation();

        t1_recovered += off;

        valid = trackPoint(pyr_2, pyr_1, transform_2, transform_1_recovered);
        if (!valid) continue;

        Scalar dist2 = (t1 - t1_recovered).squaredNorm();

        if (dist2 < config.optical_flow_max_recovered_dist2) {
          result[id] = transform_2;
        }
      }
    };

    tbb::blocked_range<size_t> range(0, num_points);
    tbb::parallel_for(range, compute_func);

    keypoint_map_2.clear();
    keypoint_map_2.insert(result.begin(), result.end());
    guesses.clear();
    guesses.insert(guesses_tbb.begin(), guesses_tbb.end());
  }

  inline bool trackPoint(const ManagedImagePyr<uint16_t>& old_pyr, const ManagedImagePyr<uint16_t>& pyr,
                         const Eigen::AffineCompact2f& old_transform, Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    transform.linear().setIdentity();

    for (int level = config.optical_flow_levels; level >= 0 && patch_valid; level--) {
      const Scalar scale = 1 << level;

      transform.translation() /= scale;

      PatchT p(old_pyr.lvl(level), old_transform.translation() / scale);

      patch_valid &= p.valid;
      if (patch_valid) {
        // Perform tracking on current level
        patch_valid &= trackPointAtLevel(pyr.lvl(level), p, transform);
      }

      transform.translation() *= scale;
    }

    transform.linear() = old_transform.linear() * transform.linear();

    return patch_valid;
  }

  inline bool trackPointAtLevel(const Image<const uint16_t>& img_2, const PatchT& dp, Eigen::AffineCompact2f& transform,
                                Scalar* out_error = nullptr) const {
    bool patch_valid = true;

    for (int iteration = 0; patch_valid && iteration < config.optical_flow_max_iterations; iteration++) {
      typename PatchT::VectorP res;

      typename PatchT::Matrix2P transformed_pat = transform.linear().matrix() * PatchT::pattern2;
      transformed_pat.colwise() += transform.translation();

      patch_valid &= dp.residual(img_2, transformed_pat, res);

      if (out_error) *out_error = res.norm();

      if (patch_valid) {
        const Vector3 inc = -dp.H_se2_inv_J_se2_T * res;

        // avoid NaN in increment (leads to SE2::exp crashing)
        patch_valid &= inc.array().isFinite().all();

        // avoid very large increment
        patch_valid &= inc.template lpNorm<Eigen::Infinity>() < 1e6;

        if (patch_valid) {
          transform *= SE2::exp(inc).matrix();

          const int filter_margin = 2;

          patch_valid &= img_2.InBounds(transform.translation(), filter_margin);
        }
      }
    }

    return patch_valid;
  }

  inline bool trackPointFromPyrPatch(const ManagedImagePyr<uint16_t>& pyr,
                                     const Eigen::aligned_vector<PatchT>& patch_vec, Eigen::AffineCompact2f& transform,
                                     std::array<size_t, 2> ts_lm = {SIZE_MAX, SIZE_MAX}) const {
    bool patch_valid = true;

    //! @note For some reason resetting transform linear part would be wrong only in patch_optical_flow?
    // transform.linear().setIdentity();

    // TODO@mateosss: values are very arbitrary from tracking lm 3 of MIO11 dataset
    std::vector<Scalar> max_error_at_level = {1.74, 0.96, 0.99, 0.44};

    for (int level = config.optical_flow_levels; level >= 0 && patch_valid; level--) {
      const Scalar scale = 1 << level;

      transform.translation() /= scale;

      // TODO: maybe we should better check patch validity when creating points
      const auto& p = patch_vec[level];
      patch_valid &= p.valid;
      if (patch_valid) {
        // Perform tracking on current level
        Scalar error = -1;
        patch_valid &= trackPointAtLevel(pyr.lvl(level), p, transform, &error);
        patch_valid &= error <= max_error_at_level.at(level);
        // TODO@mateosss: remove print
        // if (ts_lm[0] != SIZE_MAX)
        //   printf("frame:%ld,landmark:%ld,level:%d,error:%f,valid:%d\n", ts_lm[0], ts_lm[1], level, error,
        //   patch_valid);
      }

      transform.translation() *= scale;
    }

    return patch_valid;
  }

  /**
   * @brief Project the landmarks into the current frame.
   * Returns the landmarks that are in the frame along with their corresponding 2D projections.
   *
   * @param[in] cam_id: The camera id of the current frame.
   * @param[out] landmarks: A reference where landmarks will be returned.
   * @param[out] projections: A reference where the landmark's projections will be returned.
   */
  Eigen::aligned_unordered_map<LandmarkId, Vector2> getProjectedLandmarks(size_t cam_id) {
    Eigen::aligned_unordered_map<LandmarkId, Vector2> projections;

    for (const auto& [lm_id, lm_pos] : latest_lm_bundle->landmarks) {
      // Skip landmarks that are already tracked by the current frame
      // TODO@mateosss: removing already_tracked check improves tracks stability
      // with the check: we get avgdepth-reprojection+f2f OF always but when lost, it uses patch for track recovery as
      // fallback without the check: we get good-reprojection+patch OF always that overrides f2f, but when lost, it uses
      // f2f for track recovery as fallback bool already_tracked = transforms->keypoints.at(cam_id).count(lm_id) > 0; if
      // (already_tracked) continue;

      // Skip points that are behind the camera
      // TODO@mateosss: dot product between latest-predicted-state forward-vector and relative vector to landmark from
      // latest-predicted-state

      // Project landmark to current frame
      SE3 T_i1 = predicted_state->T_w_i.template cast<Scalar>();
      SE3 T_i_cj = calib.T_i_c[cam_id];
      SE3 T_cj = T_i1 * T_i_cj;
      Vector3 cj_xyz = T_cj.inverse() * lm_pos;
      Vector2 cj_uv;
      bool valid = calib.intrinsics[cam_id].project(cj_xyz, cj_uv);
      if (!valid) continue;

      // TODO@mateosss: this info should probably come from calibration
      Scalar sr = 480;
      valid &= sr == 0 || (cj_uv - Vector2{sr, sr}).norm() <= 0.95 * sr;
      if (!valid) continue;

      // Check if the point is in the bounds of the frame
      const basalt::Image<const uint16_t>& img_raw = pyramid->at(cam_id).lvl(0);
      bool in_bounds = cj_uv.x() >= 0 && cj_uv.x() < img_raw.w && cj_uv.y() >= 0 && cj_uv.y() < img_raw.h;
      if (!in_bounds) continue;

      projections[lm_id] = cj_uv;
    }

    return projections;
  }

  //! Number of points that are in the same grid cell as img_pos
  int getNeighborPointsInCell(const Vector2& img_pos, size_t cam_id) {
    const int C = config.optical_flow_detection_grid_size;
    int w = transforms->input_images->img_data.at(cam_id).img->w;
    int h = transforms->input_images->img_data.at(cam_id).img->h;
    int x_pad = (w % C) / 2;
    int y_pad = (h % C) / 2;

    const int u = img_pos.x();
    const int v = img_pos.y();
    const int cell_x = (u - x_pad) / C;
    const int cell_y = (v - y_pad) / C;

    int left = x_pad + C * cell_x;
    int top = y_pad + C * cell_y;

    int right = left + C;
    int bottom = top + C;

    int neighbors = 0;
    for (const auto& [kpid, affine] : transforms->keypoints.at(cam_id)) {
      float x = affine.translation().x();
      float y = affine.translation().y();
      if (x >= left && x < right && y >= top && y < bottom) neighbors++;
    }

    return neighbors;
  }

  //! Recover tracks from older landmarks into the current frame
  void recallPoints() {
    if (latest_lm_bundle == nullptr) return;

    uint64_t cam_id = 0;  // TODO@mateosss: recall in all cameras

    // Project the landmarks from the map into the new frame to obtain their projections.
    Eigen::aligned_unordered_map<LandmarkId, Vector2> projections = getProjectedLandmarks(cam_id);

    // TODO@mateosss: Parallelize?
    for (const auto& [lm_id, proj_pos] : projections) {
      // TODO@mateosss:
      // Decide whether to filter recalled points based on gridcell ocupancy. I think it makes sense for recalled points
      // are too important to ignore. If we want to keep the front end bounded we might remove other neighbors in that
      // cell? or just recall a limited number of points?
      //
      // This if:
      // if (getNeighborPointsInCell(old_transforms, proj_pos, cam_id) > config.optical_flow_detection_num_points_cell
      //  && getNeighborPointsInCell(new_transforms, proj_pos, cam_id) > config.optical_flow_detection_num_points_cell)
      //  continue;
      //
      // Or this if (only checks new_transforms):
      // if (getNeighborPointsInCell(proj_pos, cam_id) > config.optical_flow_detection_num_points_cell) continue;

      Eigen::AffineCompact2f proj_pose = Eigen::AffineCompact2f::Identity();
      proj_pose.translation() = proj_pos;

      Eigen::aligned_vector<PatchT>& lm_patch = patches.at(lm_id);
      Eigen::AffineCompact2f curr_pose = proj_pose;

      if (show_gui) transforms->recall_guesses.at(cam_id)[lm_id] = proj_pose;

      bool valid = trackPointFromPyrPatch(pyramid->at(cam_id), lm_patch, curr_pose, {frame_counter, lm_id});
      if (!valid) continue;

      Scalar pattern_width = 8;  // TODO@mateosss: hardcoded
      valid &= (curr_pose.translation() - proj_pos).norm() <= pattern_width * 1;
      if (!valid) continue;

#if 0
      // TODO@mateosss: check that this improves things
      // Update patch viewpoint, see point 2 of the following discussion
      // https://gitlab.com/VladyslavUsenko/basalt/-/issues/69#note_906947578
      for (int l = 0; l <= config.optical_flow_levels; l++) {
        Scalar scale = 1 << l;
        Vector2 pos_scaled = curr_pose.translation() / scale;
        lm_patch[l] = PatchT(pyramid->at(0).lvl(l), pos_scaled);
      }

      // TODO@mateosss: Maybe I should store all patches that see this landmark and look in all of them
#endif

      transforms->keypoints.at(cam_id)[lm_id] = curr_pose;
    }
  }

  Keypoints addPointsForCamera(size_t cam_id) {
    Eigen::aligned_vector<Eigen::Vector2d> pts;  // Current points
    for (const auto& [kpid, affine] : transforms->keypoints.at(cam_id)) {
      pts.emplace_back(affine.translation().template cast<double>());
    }

    KeypointsData kd;  // Detected new points
    detectKeypoints(pyramid->at(cam_id).lvl(0), kd, config.optical_flow_detection_grid_size,
                    config.optical_flow_detection_num_points_cell, config.optical_flow_detection_min_threshold,
                    config.optical_flow_detection_max_threshold, transforms->input_images->masks.at(cam_id), pts);

    Keypoints new_kpts;
    for (auto& corner : kd.corners) {  // Set new points as keypoints

      // Save patch
      Eigen::aligned_vector<PatchT>& p = patches[last_keypoint_id];
      // TODO@mateosss: check p.valid here? otherwise it is checked elsewhere
      Vector2 pos = corner.cast<Scalar>();
      for (int l = 0; l <= config.optical_flow_levels; l++) {
        Scalar scale = 1 << l;
        Vector2 pos_scaled = pos / scale;
        p.emplace_back(pyramid->at(0).lvl(l), pos_scaled);
      }

      auto transform = Eigen::AffineCompact2f::Identity();
      transform.translation() = corner.cast<Scalar>();

      transforms->keypoints.at(cam_id)[last_keypoint_id] = transform;
      new_kpts[last_keypoint_id] = transform;

      last_keypoint_id++;
    }

    return new_kpts;
  }

  Masks cam0OverlapCellsMasksForCam(size_t cam_id) {
    int C = config.optical_flow_detection_grid_size;  // cell size

    int w = transforms->input_images->img_data.at(cam_id).img->w;
    int h = transforms->input_images->img_data.at(cam_id).img->h;

    int x_start = (w % C) / 2;
    int y_start = (h % C) / 2;

    int x_stop = x_start + C * (w / C - 1);
    int y_stop = y_start + C * (h / C - 1);

    int x_first = x_start + C / 2;
    int y_first = y_start + C / 2;

    int x_last = x_stop + C / 2;
    int y_last = y_stop + C / 2;

    Masks masks;
    for (int y = y_first; y <= y_last; y += C) {
      for (int x = x_first; x <= x_last; x += C) {
        Vector2 ci_uv{x, y};
        Vector2 c0_uv;
        Scalar _;
        bool projected = calib.projectBetweenCams(ci_uv, depth_guess, c0_uv, _, cam_id, 0);
        bool in_bounds = c0_uv.x() >= 0 && c0_uv.x() < w && c0_uv.y() >= 0 && c0_uv.y() < h;
        bool valid = projected && in_bounds;
        if (valid) {
          Rect cell_mask(x - C / 2, y - C / 2, C, C);
          masks.masks.push_back(cell_mask);
        }
      }
    }
    return masks;
  }

  void addPoints() {
    Masks& ms0 = transforms->input_images->masks.at(0);
    Keypoints kpts0 = addPointsForCamera(0);

    for (size_t i = 1; i < getNumCams(); i++) {
      Masks& ms = transforms->input_images->masks.at(i);
      Keypoints& mgs = transforms->matching_guesses.at(i);

      // Match features on areas that overlap with cam0 using optical flow
      auto& pyr0 = pyramid->at(0);
      auto& pyri = pyramid->at(i);
      Keypoints kpts;
      SE3 T_c0_ci = calib.T_i_c[0].inverse() * calib.T_i_c[i];
      trackPoints(pyr0, pyri, kpts0, kpts, mgs, ms0, ms, T_c0_ci, 0, i);
      transforms->keypoints.at(i).insert(kpts.begin(), kpts.end());

      // Update masks and detect features on area not overlapping with cam0
      if (!config.optical_flow_detection_nonoverlap) continue;
      ms += cam0OverlapCellsMasksForCam(i);
      Keypoints kpts_no = addPointsForCamera(i);
    }
  }

  void filterPointsForCam(int cam_id) {
    std::set<KeypointId> kp_to_remove;

    std::vector<KeypointId> kpids;
    Eigen::aligned_vector<Eigen::Vector2f> proj0, proj1;

    for (const auto& [kpid, affine] : transforms->keypoints.at(cam_id)) {
      auto it = transforms->keypoints.at(0).find(kpid);

      if (it != transforms->keypoints.at(0).end()) {
        proj0.emplace_back(it->second.translation());
        proj1.emplace_back(affine.translation());
        kpids.emplace_back(kpid);
      }
    }

    Eigen::aligned_vector<Eigen::Vector4f> p3d0, p3d1;
    std::vector<bool> p3d0_success, p3d1_success;

    calib.intrinsics[0].unproject(proj0, p3d0, p3d0_success);
    calib.intrinsics[cam_id].unproject(proj1, p3d1, p3d1_success);

    for (size_t i = 0; i < p3d0_success.size(); i++) {
      if (p3d0_success[i] && p3d1_success[i]) {
        const double epipolar_error = std::abs(p3d0[i].transpose() * E[cam_id] * p3d1[i]);

        if (epipolar_error > config.optical_flow_epipolar_error) {
          kp_to_remove.emplace(kpids[i]);
        }
      } else {
        kp_to_remove.emplace(kpids[i]);
      }
    }

    for (int id : kp_to_remove) {
      transforms->keypoints.at(cam_id).erase(id);
    }
  }

  void filterPoints() {
    for (size_t i = 1; i < getNumCams(); i++) {
      filterPointsForCam(i);
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::aligned_unordered_map<KeypointId, Eigen::aligned_vector<PatchT>> patches;
  const Vector3d accel_cov;
  const Vector3d gyro_cov;
};

}  // namespace basalt
