/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <core/trajectory_manager.h>
#include <utils/math_utils.h>
#include <utils/eigen_utils.hpp>
#include <utils/ceres_callbacks.h>

#include <memory>

namespace licalib {
using namespace kontiki::trajectories;

void TrajectoryManager::initialTrajTo(double max_time) {
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p0(0,0,0);
  traj_->R3Spline()->ExtendTo (max_time, p0);
  traj_->SO3Spline()->ExtendTo(max_time, q0);
}

void TrajectoryManager::feedIMUData(const IO::IMUData& data) {
  imu_data_.emplace_back(data);
}

void TrajectoryManager::initialSO3TrajWithGyro() {
  assert(imu_data_.size() > 0 &&
         "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");
  std::shared_ptr<SO3TrajEstimator> estimator_SO3;
  estimator_SO3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline());

  addGyroscopeMeasurements(estimator_SO3);

  /// fix the initial pose of trajectory
  double weight_t0 = calib_param_manager->global_opt_gyro_weight;
  double t0 = traj_->SO3Spline()->MinTime();
  //Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0,0,1));
  Eigen::Quaterniond q0 = Eigen::Quaterniond (rotation_vector.matrix());
  auto m_q0 = std::make_shared<OrientationMeasurement>(t0, q0, weight_t0);
  estimator_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

  ceres::Solver::Summary summary = estimator_SO3->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
}

void TrajectoryManager::trajInitFromSurfel(
        SurfelAssociation::Ptr surfels_association,
        bool opt_time_offset_) {
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI);
  lidar_->LockRelativeOrientation(false);
  lidar_->LockRelativePosition(false);
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    lidar_->LockTimeOffset(false);
    lidar_->set_max_time_offset(time_offset_padding_);
  }
  else {
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  addSurfMeasurement(estimator_split, surfels_association);

  // addCallback(estimator_split);

  //printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(lidar_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates();
}

bool TrajectoryManager::evaluateIMUPose(double imu_time, int flags,
                                        Result &result) const {
  if (traj_->MinTime() > imu_time || traj_->MaxTime() <= imu_time)
    return false;
  result = traj_->Evaluate(imu_time, flags);
  return true;
}

bool TrajectoryManager::evaluateLidarPose(double lidar_time,
                                          Eigen::Quaterniond &q_LtoG,
                                          Eigen::Vector3d &p_LinG) const {
  double traj_time = lidar_time + lidar_->time_offset();
  if (traj_->MinTime() > traj_time || traj_->MaxTime() <= traj_time)
    return false;
  Result result = traj_->Evaluate( traj_time, EvalOrientation | EvalPosition);
  q_LtoG = result->orientation * calib_param_manager->q_LtoI;
  p_LinG = result->orientation * calib_param_manager->p_LinI + result->position;
  return true;
}

bool TrajectoryManager::evaluateLidarRelativeRotation(double lidar_time1,
        double lidar_time2, Eigen::Quaterniond &q_L2toL1) const {
  assert(lidar_time1 <= lidar_time2
         && "[evaluateRelativeRotation] : lidar_time1 > lidar_time2");

  double traj_time1 = lidar_time1 + lidar_->time_offset();
  double traj_time2 = lidar_time2 + lidar_->time_offset();

  if (traj_->MinTime() > traj_time1 || traj_->MaxTime() <= traj_time2)
    return false;

  Result result1 = traj_->Evaluate(traj_time1, EvalOrientation);
  Result result2 = traj_->Evaluate(traj_time2, EvalOrientation);
  Eigen::Quaterniond q_I2toI1 = result1->orientation.conjugate()*result2->orientation;

  q_L2toL1 = calib_param_manager->q_LtoI.conjugate() * q_I2toI1 * calib_param_manager->q_LtoI;
  return true;
}

template <typename TrajectoryModel>
void TrajectoryManager::addGyroscopeMeasurements(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  gyro_list_.clear();

  double weight = calib_param_manager->global_opt_gyro_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();

  for (const auto &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    auto mg = std::make_shared<GyroMeasurement>(imu_, v.timestamp, v.gyro, weight);
    gyro_list_.push_back(mg);
    estimator->template AddMeasurement<GyroMeasurement>(mg);
  }
}

template <typename TrajectoryModel>
void TrajectoryManager::addAccelerometerMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  accel_list_.clear();

  const double weight = calib_param_manager->global_opt_acce_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();

  for (auto const &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    auto ma = std::make_shared<AccelMeasurement>(imu_, v.timestamp, v.accel, weight);
    accel_list_.push_back(ma);
    estimator->template AddMeasurement<AccelMeasurement>(ma);
  }
}

template <typename TrajectoryModel>
void TrajectoryManager::addSurfMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
        const SurfelAssociation::Ptr surfel_association) {
  const double weight = calib_param_manager->global_opt_lidar_weight;
  surfelpoint_list_.clear();
  closest_point_vec_.clear();
  for (auto const& v: surfel_association->get_surfel_planes()) {
    closest_point_vec_.push_back(v.Pi);
  }

  map_time_ = surfel_association->get_maptime();
  for (auto const &spoint : surfel_association->get_surfel_points()) {
    double time = spoint.timestamp;
    size_t plane_id = spoint.plane_id;

    auto msp = std::make_shared<SurfMeasurement> (lidar_, spoint.point,
                                                  closest_point_vec_.at(plane_id).data(), time, map_time_, 5.0, weight);
    surfelpoint_list_.push_back(msp);
    estimator->template AddMeasurement<SurfMeasurement>(msp);
  }
}

template <typename TrajectoryModel>
void TrajectoryManager::addCallback(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  // Add callback for debug
  std::unique_ptr<CheckStateCallback> cb  = std::make_unique<CheckStateCallback>();
  cb->addCheckState("q_LtoI     :", 4, lidar_->relative_orientation().coeffs().data());
  cb->addCheckState("p_LinI     :", 3, lidar_->relative_position().data());
  cb->addCheckState("time_offset:", 1, &lidar_->time_offset());
  cb->addCheckState("g_roll     :", 1, &imu_->gravity_orientation_roll());
  cb->addCheckState("g_pitch    :", 1, &imu_->gravity_orientation_pitch());
  estimator->AddCallback(std::move(cb), true);
}

void TrajectoryManager::printErrorStatistics(const std::string& intro, bool show_gyro,
                                             bool show_accel, bool show_lidar) const {
  std::cout << "\n============== " << intro << " ================" << std::endl;

  if (show_gyro && !gyro_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : gyro_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    std::cout << "[Gyro]  Error size, average: " << gyro_list_.size()
              << "; " << (error_sum/gyro_list_.size()).transpose() << std::endl;
  }

  if (show_accel && !accel_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : accel_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    std::cout << "[Accel] Error size, average: " << accel_list_.size()
              << ";  " << (error_sum/accel_list_.size()).transpose() << std::endl;
  }

  if (show_lidar && !surfelpoint_list_.empty()) {
    Eigen::Matrix<double,1,1>  error_sum;
    for (auto const &m : surfelpoint_list_) {
      error_sum += m->point2plane<SplitTrajectory>(*traj_).cwiseAbs();
    }
    std::cout << "[LiDAR] Error size, average: " << surfelpoint_list_.size()
              << "; " << (error_sum/surfelpoint_list_.size()).transpose() << std::endl;
  }

  std::cout << std::endl;
}

}