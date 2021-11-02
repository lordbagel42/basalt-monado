// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SLAM tracker class header for usage in Monado.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 *
 * This header file contains the declaration of the @ref slam_tracker class to
 * implement a SLAM system for usage in Monado.
 *
 * A copy of this file is present in both Monado and any SLAM system that
 * intends to be used by Monado. The SLAM system should provide the
 * `slam_tracker` implementation so that Monado can use it.
 *
 * This file also declares additional data types to be used between Monado and
 * the system.
 *
 * @todo The interface is preliminary and should be improved to avoid
 * unnecessary copies.
 */

#pragma once

#include <opencv2/core/mat.hpp>

namespace xrt::auxiliary::tracking::slam {

/*!
 * @brief Standard pose type to communicate Monado with the external SLAM system
 */
struct pose {
  float px, py, pz;
  float rx, ry, rz, rw;
  pose() = default;
  pose(float px, float py, float pz, float rx, float ry, float rz, float rw)
      : px(px), py(py), pz(pz), rx(rx), ry(ry), rz(rz), rw(rw) {}
};

/*!
 * @brief IMU Sample type to pass around between programs
 */
struct imu_sample {
  std::int64_t timestamp; //!< In nanoseconds
  double ax, ay, az;      //!< Accel in meters per second squared (m / s^2)
  double wx, wy, wz;      //!< Gyro in radians per second (rad / s)
  imu_sample() = default;
  imu_sample(std::int64_t timestamp, double ax, double ay, double az, double wx,
             double wy, double wz)
      : timestamp(timestamp), ax(ax), ay(ay), az(az), wx(wx), wy(wy), wz(wz) {}
};

/*!
 * @brief Image sample type to pass around between programs. It is expected that
 * any SLAM system takes OpenCV matrices as input.
 */
struct img_sample {
  std::int64_t timestamp;
  cv::Mat img;
  bool is_left;
  img_sample() = default;
  img_sample(std::int64_t timestamp, cv::Mat img, bool is_left)
      : timestamp(timestamp), img(img), is_left(is_left) {}
};

/*!
 * @brief slam_tracker serves as an interface between Monado and external SLAM
 * systems.
 *
 * This class uses the pointer-to-implementation pattern, and its implementation
 * should be provided by an external SLAM system.
 */
struct slam_tracker {
  /*!
   * @brief Construct a new slam tracker object
   *
   * @param config_file SLAM systems parameters tend to be numerous and very
   * specific, so they usually use a configuration file as the main way to set
   * them up. Therefore, this constructor receives a path to a
   * implementation-specific configuration file.
   */
  slam_tracker(std::string config_file);
  ~slam_tracker();

  slam_tracker(const slam_tracker &) = delete;
  slam_tracker &operator=(const slam_tracker &) = delete;

  void initialize();
  void start();
  bool is_running();
  void stop();
  void finalize();

  //! There must be a single producer thread pushing samples.
  //! Samples must have monotonically increasing timestamps.
  //! The implementation must be non-blocking.
  //! A separate consumer thread should process the samples.
  void push_imu_sample(imu_sample sample);

  //! Same conditions as `push_imu_sample` apply.
  //! When using stereo frames, they must be pushed in a left-right order.
  //! The consecutive left-right pair must have the same timestamps.
  void push_frame(img_sample sample);

  //! There must be a single thread accessing the tracked pose.
  bool try_dequeue_pose(pose &pose);

  // TODO@mateosss: document
  bool supports_feature(int feature_id);
  // TODO@mateosss: bool says whether feature_id is supported
  bool use_feature(int feature_id, std::shared_ptr<void> params,
                   std::shared_ptr<void> &out);

private:
  struct implementation;
  // TODO@mateosss: use unique_ptr
  implementation *impl;
};

// Features
// TODO@mateosss: feature documentation, use before initialize
// Feature 1: add_cam_calibration
struct cam_calibration {
  enum class cam_model { pinhole, fisheye };

  int cam_index; //!< For multi-camera setups. For stereo 0 ~ left, 1 ~ right.
  int width, height; //<! Resolution
  int fps;           //<! Frames per second
  double fx, fy;     //<! Focal point
  double cx, cy;     //<! Principal point
  cam_model model;
  std::vector<double> model_params;
  cv::Matx<double, 4, 4> T_cam_imu; //!< Transformation from camera to imu space
};

// TODO@mateosss: macro? short: ACC

using FPARAMS_ACC = cam_calibration;
using FRESULT_ACC = void;
constexpr int FID_ACC = 1;
constexpr int F_ADD_CAMERA_CALIBRATION = 1;

} // namespace xrt::auxiliary::tracking::slam
