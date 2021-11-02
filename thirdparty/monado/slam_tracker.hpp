// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SLAM tracker class header for usage in Monado.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 *
 * This file contains the declaration of the @ref slam_tracker class. This
 * header is intended to appear in both Monado and an external SLAM system. The
 * implementation of `slam_tracker` is provided by the external system.
 * Additional data types are declared for the communication between Monado and
 * the system.
 *
 * TODO@mateosss: maybe use const&
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

  /*!
   * @brief Push an IMU sample into the tracker.
   *
   * There must be a single producer thread pushing samples.
   * Samples must have monotonically increasing timestamps.
   * The implementation must be non-blocking.
   * Thus, a separate consumer thread should process the samples.
   */
  void push_imu_sample(imu_sample sample);

  /*!
   * @brief Push an image sample into the tracker.
   *
   * Same conditions as @ref push_imu_sample apply.
   * When using stereo frames, they must be pushed in a left-right order.
   * The consecutive left-right pair must have the same timestamps.
   */
  void push_frame(img_sample sample);

  /*!
   * @brief Get the latest tracked pose from the SLAM system.
   *
   * There must be a single thread consuming this method.
   *
   * @param[out] pose Dequeued pose.
   * @return true If a new pose was dequeued into @p pose.
   * @return false If there was no pose to dequeue.
   */
  bool try_dequeue_pose(pose &pose);

  //! Asks the SLAM system whether it supports a specific feature.
  bool supports_feature(int feature_id);

  /*!
   * @brief Use a special feature of the SLAM tracker.
   *
   * This method uses heap allocated objects for passing parameters and
   * obtaining the results. Use `std::static_pointer_cast` to shared pointers to
   * the expected types.
   *
   * @param feature_id Id of the special feature.
   * @param params Pointer to the parameter object for this feature.
   * @param result Pointer to the result produced by the feature call.
   * @return false if the feature was not supported, true otherwise.
   */
  bool use_feature(int feature_id, std::shared_ptr<void> params,
                   std::shared_ptr<void> &result);

private:
  struct implementation;
  // TODO@mateosss: use unique_ptr
  implementation *impl;
};

/*
 * Special features
 *
 * A special feature is comprised of an ID, a PARAMS type and a RESULT type. It
 * can be defined using DEFINE_FEATURE. Once defined, the definition should not
 * suffer future changes.
 *
 * One of the main concerns in the features interface is the ability to add new
 * features without being required to update the SLAM systems that are not
 * interested in implementing the feature.
 *
 */

#define DEFINE_FEATURE(NAME, SHORT_NAME, ID, PARAMS_TYPE, RESULT_TYPE)         \
  using FPARAMS_##SHORT_NAME = PARAMS_TYPE;                                    \
  using FRESULT_##SHORT_NAME = RESULT_TYPE;                                    \
  constexpr int FID_##SHORT_NAME = ID;                                         \
  constexpr int F_##NAME = ID;

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

/*!
 * Feature ADD_CAMERA_CALIBRATION
 *
 * Use it after constructor but before `start()` to write or overwrite camera
 * calibration data that might come from the system-specific config file.
 */
DEFINE_FEATURE(ADD_CAMERA_CALIBRATION, ACC, 1, cam_calibration, void)

} // namespace xrt::auxiliary::tracking::slam
