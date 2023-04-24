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
#ifndef SURFELASSOCIATION_HPP
#define SURFELASSOCIATION_HPP

#include <pclomp/ndt_omp.h>
#include <utils/vlp_common.h>
#include <boost/circular_buffer.hpp>
#include <utils/eigen_utils.hpp>

namespace licalib {

class SurfelAssociation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<SurfelAssociation> Ptr;

  struct SurfelPoint {
    double timestamp;
    Eigen::Vector3d point;  // raw data
    Eigen::Vector3d point_in_map;
    size_t plane_id;
  };

  struct SurfelPlane {
    Eigen::Vector4d p4;
    Eigen::Vector3d Pi; // Closest Point Paramization
    Eigen::Vector3d boxMin;
    Eigen::Vector3d boxMax;
    VPointCloud cloud;
    VPointCloud cloud_inlier;
  };

  explicit SurfelAssociation(double associated_radius = 0.05,
                             double plane_lambda = 0.7)
          : associated_radius_(associated_radius),
            p_lambda_(plane_lambda),
            map_timestamp_(0) {
    initColorList();
  }

  void setSurfelMap(const pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr& ndtPtr,
                    double timestamp = 0);

  void setPlaneLambda(double lambda) {
    p_lambda_ = lambda;
  }

  void getAssociation(const VPointCloud::Ptr& scan_inM,
                      const TPointCloud::Ptr& scan_raw,
                      size_t selected_num_per_ring = 2);

  void randomDownSample(int num_points_max = 5);

  void averageDownSmaple(int num_points_max = 5);

  void averageTimeDownSmaple(int step = 10);

  const Eigen::aligned_vector<SurfelPlane>& get_surfel_planes() const {
    return surfel_planes_;
  }

  const Eigen::aligned_vector<SurfelPoint>& get_surfel_points() const {
    return spoint_downsampled_;
  }

  double get_maptime() const {
    return map_timestamp_;
  }

  void saveSurfelsMap(std::string& path) const {
    std::cout << "Save surfel map to " << path
              << "; size: " << surfels_map_.size() << std::endl;
    pcl::io::savePCDFileASCII(path, surfels_map_);
  }

private:
  void initColorList();

  void clearSurfelMap();

  static int checkPlaneType(const Eigen::Vector3d& eigen_value,
                            const Eigen::Matrix3d& eigen_vector,
                            const double& p_lambda);

  static bool fitPlane(const VPointCloud::Ptr& cloud,
                       Eigen::Vector4d &coeffs,
                       VPointCloud::Ptr cloud_inliers);

  static double point2PlaneDistance(Eigen::Vector3d &pt,
                                    Eigen::Vector4d &plane_coeff);

  void associateScanToSurfel(const size_t& surfel_idx,
                             const VPointCloud::Ptr& scan,
                             const double& radius,
                             std::vector<std::vector<int>> &ring_masks) const;

private:

  boost::circular_buffer<int> color_list_; // for visualization

  double associated_radius_;
  double p_lambda_;
  double map_timestamp_;

  Eigen::aligned_vector<SurfelPlane> surfel_planes_;
  colorPointCloudT surfels_map_;

  // associated results
  Eigen::aligned_vector<Eigen::aligned_vector<SurfelPoint>> spoint_per_surfel_;
  Eigen::aligned_vector<SurfelPoint> spoints_all_;

  Eigen::aligned_vector<SurfelPoint> spoint_downsampled_;
};

}


#endif
