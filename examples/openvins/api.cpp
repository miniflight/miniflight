#include "core/VioManager.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include <cassert>
#include <deque>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

struct Observer {
  std::unique_ptr<ov_msckf::VioManager> estimator;
  std::deque<ov_core::CameraData> cameras;
  double imu_time = -1;
  explicit Observer(const char *config) {
    ov_core::Printer::setPrintLevel("WARNING");
    auto parser = std::make_shared<ov_core::YamlParser>(config);
    ov_msckf::VioManagerOptions options;
    options.print_and_load(parser);
    options.use_multi_threading_subs = false;
    options.use_multi_threading_pubs = false;
    options.num_opencv_threads = 1;
    if (!parser->successful())
      throw std::runtime_error("invalid VIO configuration");
    estimator = std::make_unique<ov_msckf::VioManager>(options);
  }
  void drain() {
    while (!cameras.empty() &&
           imu_time >
               cameras.front().timestamp +
                   estimator->get_state()->_calib_dt_CAMtoIMU->value()(0)) {
      auto sample = cameras.front();
      cameras.pop_front();
      estimator->feed_measurement_camera(sample);
    }
  }
};
extern "C" {
void *vio_create(const char *config) {
  try {
    return new Observer(config);
  } catch (...) {
    return nullptr;
  }
}
void vio_destroy(void *observer) { delete static_cast<Observer *>(observer); }
int vio_imu(void *observer, double time, const double *gyro,
            const double *acceleration) try {
  auto &o = *static_cast<Observer *>(observer);
  ov_core::ImuData sample;
  sample.timestamp = time;
  sample.wm = Eigen::Map<const Eigen::Vector3d>(gyro);
  sample.am = Eigen::Map<const Eigen::Vector3d>(acceleration);
  o.estimator->feed_measurement_imu(sample);
  o.imu_time = time;
  o.drain();
  return 1;
} catch (...) {
  return 0;
}
int vio_camera(void *observer, double time, const unsigned char *pixels,
               int width, int height) try {
  auto &o = *static_cast<Observer *>(observer);
  ov_core::CameraData sample;
  sample.timestamp = time;
  sample.sensor_ids = {0};
  sample.images = {
      cv::Mat(height, width, CV_8UC1, const_cast<unsigned char *>(pixels))
          .clone()};
  sample.masks = {cv::Mat::zeros(height, width, CV_8UC1)};
  o.cameras.push_back(sample);
  o.drain();
  return 1;
} catch (...) {
  return 0;
}
int vio_state(void *observer, double *output) {
  auto &o = *static_cast<Observer *>(observer);
  auto s = o.estimator->get_state();
  if (s->_timestamp < 0 ||
      o.imu_time < s->_timestamp + s->_calib_dt_CAMtoIMU->value()(0))
    return 0;
  Eigen::Matrix<double, 13, 1> state;
  Eigen::Matrix<double, 12, 12> covariance;
  auto propagator = o.estimator->get_propagator();
  propagator->invalidate_cache();
  // A fresh cache applies camera-to-IMU offset to both integration endpoints.
  if (!propagator->fast_state_propagate(
          s, o.imu_time - s->_calib_dt_CAMtoIMU->value()(0), state, covariance))
    return 0;
  Eigen::Matrix3d R = ov_core::quat_2_Rot(state.head<4>()).transpose();
  Eigen::Vector3d velocity = R * state.segment<3>(7);
  output[0] = o.imu_time;
  output[1] = s->_timestamp + s->_calib_dt_CAMtoIMU->value()(0);
  for (int i = 0; i < 3; i++) {
    output[2 + i] = state(4 + i);
    output[5 + i] = velocity(i);
  }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      output[8 + 3 * i + j] = R(i, j);
  for (int i = 0; i < 3; i++) {
    output[17 + i] = s->_imu->bias_g()(i);
    output[20 + i] = s->_imu->bias_a()(i);
  }
  output[23] = s->_calib_dt_CAMtoIMU->value()(0);
  output[24] = s->_clones_IMU.size();
  return 1;
}
}
