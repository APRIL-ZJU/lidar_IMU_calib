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
#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Eigen>
#include <memory>
#include <fstream>

namespace licalib {

class CalibParamManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<CalibParamManager> Ptr;

  CalibParamManager() :
          p_LinI(Eigen::Vector3d(0,0,0)) ,
          q_LtoI(Eigen::Quaterniond::Identity()),
          gravity(Eigen::Vector3d(0, 0, -9.8)),
          time_offset(0),
          gyro_bias(Eigen::Vector3d(0,0,0)),
          acce_bias(Eigen::Vector3d(0,0,0)) {

    // fine-tuned parameter
    global_opt_gyro_weight = 28.0;
    global_opt_acce_weight = 18.5;
    global_opt_lidar_weight = 10.0;
  }

  void set_q_LtoI(Eigen::Quaterniond q) {
    q_LtoI = q;
  }

  void set_p_LinI(Eigen::Vector3d p) {
    p_LinI = p;
  }

  void set_gravity(Eigen::Vector3d g) {
    gravity = g;
  }

  void set_time_offset(double t) {
    time_offset = t;
  }

  void set_gyro_bias(Eigen::Vector3d gb) {
    gyro_bias = gb;
  }

  void set_acce_bias(Eigen::Vector3d ab) {
    acce_bias = ab;
  }

  void set_opt_weights(const double gyroscope_noise,
                       const double accelerometer_noise,
                       const double lidar_noise,
                       const double imu_rate) {
    double gyro_discrete = gyroscope_noise * std::sqrt(imu_rate);
    double acce_discrete = accelerometer_noise * std::sqrt(imu_rate);

    global_opt_gyro_weight = 1.0 / std::pow(gyro_discrete, 2);
    global_opt_acce_weight = 1.0 / std::pow(acce_discrete, 2);
    global_opt_lidar_weight = 1.0 / std::pow(lidar_noise, 2);
  }

  bool areSensorParamsDefault(const double gyroscope_noise,
                              const double accelerometer_noise,
                              const double lidar_noise,
                              const double imu_rate,
                              const double tol = 1e-6) {
    return( (fabs(default_gyro_noise_density_ - gyroscope_noise) < tol) &&
            (fabs(default_accel_noise_density_ - accelerometer_noise) < tol) &&
            (fabs(default_lidar_noise_ - lidar_noise) < tol) &&
            (fabs(default_imu_rate_ - imu_rate) < tol) );
  }

  void showStates() const {
    Eigen::Vector3d euler_LtoI = q_LtoI.toRotationMatrix().eulerAngles(0,1,2);
    euler_LtoI = euler_LtoI * 180 / M_PI;

    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();
    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);
    Eigen::Vector3d euler_ItoL = q_ItoL.toRotationMatrix().eulerAngles(0,1,2);
    euler_ItoL = euler_ItoL * 180 / M_PI;

    std::cout << "P_LinI      : " << p_LinI.transpose() << std::endl;
    std::cout << "euler_LtoI  : " << euler_LtoI.transpose() << std::endl;
    std::cout << "P_IinL      : " << p_IinL.transpose() << std::endl;
    std::cout << "euler_ItoL  : " << euler_ItoL.transpose() << std::endl;
    std::cout << "time offset : " << time_offset << std::endl;
    std::cout << "gravity     : " << gravity.transpose() << std::endl;
    std::cout << "acce bias   : " << acce_bias.transpose() << std::endl;
    std::cout << "gyro bias   : " << gyro_bias.transpose() << std::endl;
  }

  void save_result(const std::string& filename, const std::string& info) const {
    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();
    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);

    std::ofstream outfile;
    outfile.open(filename, std::ios::app);
    outfile << info << ","
            << p_IinL(0) << "," << p_IinL(1) << "," << p_IinL(2) << ","
            << q_ItoL.x() << "," << q_ItoL.y() << "," << q_ItoL.z() << "," << q_ItoL.w() << ","
            << time_offset << "," << gravity(0) << "," << gravity(1) << "," << gravity(2) << ","
            << gyro_bias(0) << "," << gyro_bias(1) << "," <<gyro_bias(2) << ","
            << acce_bias(0) << "," << acce_bias(1) << "," <<acce_bias(2) << "\n";
    outfile.close();
  }

public:
  Eigen::Vector3d p_LinI;

  Eigen::Quaterniond q_LtoI;

  Eigen::Vector3d gravity;

  double time_offset;

  Eigen::Vector3d gyro_bias;

  Eigen::Vector3d acce_bias;

  ///default sensor parameters
  double default_gyro_noise_density_ = 1.745e-4;

  double default_accel_noise_density_ = 5.88e-4;

  double default_lidar_noise_ = 0.02;

  double default_imu_rate_ = 400.0;

  ///weight
  double global_opt_gyro_weight;

  double global_opt_acce_weight;

  double global_opt_lidar_weight;
};

}

#endif
