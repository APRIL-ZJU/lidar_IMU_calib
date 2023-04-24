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
#ifndef CALIBR_INERTIALINITIALIZER_H
#define CALIBR_INERTIALINITIALIZER_H

#include <ros/ros.h>
#include <utils/eigen_utils.hpp>
#include <core/trajectory_manager.h>
#include <core/lidar_odometry.h>

namespace licalib {


class InertialInitializer {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<InertialInitializer> Ptr;

  explicit InertialInitializer() : rotaion_initialized_(false),
                                   q_ItoS_est_(Eigen::Quaterniond::Identity()) {
  }

  bool EstimateRotation(TrajectoryManager::Ptr traj_manager,
                        const Eigen::aligned_vector<LiDAROdometry::OdomData>& odom_data);

  bool isInitialized() {
    return rotaion_initialized_;
  }

  Eigen::Quaterniond getQ_ItoS() {
    return q_ItoS_est_;
  }


private:
  bool rotaion_initialized_;
  Eigen::Quaterniond q_ItoS_est_;

};


}

#endif //CALIBR_INERTIALINITIALIZER_H
