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
#ifndef _CERES_CALLBACKS_
#define _CERES_CALLBACKS_

#include <ceres/ceres.h>
#include <ceres/iteration_callback.h>
#include <ceres/internal/port.h>


namespace licalib {

class CheckStateCallback : public ceres::IterationCallback {
public:
  CheckStateCallback() : iteration_(0u) {}

  ~CheckStateCallback() {}

  void addCheckState(const std::string& description, size_t block_size,
                     double* param_block) {
    parameter_block_descr.push_back(description);
    parameter_block_sizes.push_back(block_size);
    parameter_blocks.push_back(param_block);
  }

  ceres::CallbackReturnType operator()(
          const ceres::IterationSummary& summary) {
    std::cout << "Iteration: " << iteration_ << std::endl;
    for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
      std::cout << parameter_block_descr.at(i) << " ";
      for (size_t k = 0; k < parameter_block_sizes.at(i); ++k)
        std::cout << parameter_blocks.at(i)[k] << " ";
      std::cout << std::endl;
    }

    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }

private:

  std::vector<std::string> parameter_block_descr;
  std::vector<size_t>      parameter_block_sizes;
  std::vector<double*>     parameter_blocks;

  // Count iterations locally
  size_t iteration_;
};

}


#endif
