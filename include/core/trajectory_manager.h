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
#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <kontiki/sensors/constant_bias_imu.h>
#include <kontiki/sensors/vlp16_lidar.h>
#include <kontiki/trajectory_estimator.h>
#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
#include <kontiki/measurements/lidar_surfel_point.h>
#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/position_measurement.h>

#include <utils/dataset_reader.h>
#include <core/surfel_association.h>
#include <core/calibration.hpp>
#include <Eigen/Core>
#include <fstream>
#include <memory>

namespace licalib {
class TrajectoryManager {
  using IMUSensor = kontiki::sensors::ConstantBiasImu;
  using LiDARSensor = kontiki::sensors::VLP16LiDAR;

  using SO3TrajEstimator   = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformSO3SplineTrajectory>;
  using R3TrajEstimator    = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
  using SplitTrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::SplitTrajectory>;

  using GyroMeasurement    = kontiki::measurements::GyroscopeMeasurement<IMUSensor>;
  using AccelMeasurement   = kontiki::measurements::AccelerometerMeasurement<IMUSensor>;
  using SurfMeasurement         = kontiki::measurements::LiDARSurfelPoint<LiDARSensor>;

  using OrientationMeasurement  = kontiki::measurements::OrientationMeasurement;
  using PositionMeasurement     = kontiki::measurements::PositionMeasurement;

public:
  typedef std::shared_ptr<TrajectoryManager> Ptr;
  using Result = std::unique_ptr<kontiki::trajectories::TrajectoryEvaluation<double>>;

  explicit TrajectoryManager(double start_time, double end_time,
                             double knot_distance = 0.02,
                             double time_offset_padding = 0)
          : time_offset_padding_(time_offset_padding),
            map_time_(0),
            imu_(std::make_shared<IMUSensor>()),
            lidar_(std::make_shared<LiDARSensor>()),
            calib_param_manager(std::make_shared<CalibParamManager>()) {
    assert(knot_distance > 0 && "knot_distance should be lager than 0");

    double traj_start_time = start_time - time_offset_padding;
    double traj_end_time = end_time + time_offset_padding;
    traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>
            (knot_distance, knot_distance, traj_start_time, traj_start_time);
    initialTrajTo(traj_end_time);
  }

  void initialTrajTo(double max_time);

  void feedIMUData(const IO::IMUData& data);

  void initialSO3TrajWithGyro();

  void trajInitFromSurfel(SurfelAssociation::Ptr surfels_association,
                          bool opt_time_offset_ = false);

  bool evaluateIMUPose(double imu_time, int flags, Result& result) const;

  bool evaluateLidarPose(double lidar_time, Eigen::Quaterniond& q_LtoG,
                         Eigen::Vector3d& p_LinG) const;

  bool evaluateLidarRelativeRotation(double lidar_time1, double lidar_time2,
                                     Eigen::Quaterniond& q_L2toL1) const;

  CalibParamManager::Ptr getCalibParamManager() const {
    return calib_param_manager;
  }

  double get_map_time() const {
    return map_time_;
  }

  /// Access the trajectory
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> getTrajectory() const {
    return traj_;
  }

private:
  template <typename TrajectoryModel>
  void addGyroscopeMeasurements(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  template <typename TrajectoryModel>
  void addAccelerometerMeasurement(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  template <typename TrajectoryModel>
  void addSurfMeasurement(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
          const SurfelAssociation::Ptr surfel_association);

  template <typename TrajectoryModel>
  void addCallback(
          std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator);

  void printErrorStatistics(const std::string& intro, bool show_gyro = true,
                            bool show_accel = true, bool show_lidar = true) const;

  double map_time_;
  double time_offset_padding_;
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_;
  std::shared_ptr<kontiki::sensors::ConstantBiasImu> imu_;
  std::shared_ptr<kontiki::sensors::VLP16LiDAR> lidar_;

  CalibParamManager::Ptr calib_param_manager;

  std::vector<IO::IMUData> imu_data_;

  Eigen::aligned_vector<Eigen::Vector3d> closest_point_vec_;

  std::vector< std::shared_ptr<GyroMeasurement>>  gyro_list_;
  std::vector< std::shared_ptr<AccelMeasurement>> accel_list_;
  std::vector< std::shared_ptr<SurfMeasurement>>  surfelpoint_list_;
};
}

#endif
