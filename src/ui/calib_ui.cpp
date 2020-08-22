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
#include <ui/calib_ui.h>

const int SCAN_LENGTH = 300;

std::ostream& operator << (std::ostream& out, const TranslationVector& t) {
  out<<"=["<<std::setprecision(4)<<t.trans(0)<<','<<t.trans(1)<<','<<t.trans(2)<<"]";
  return out;
}

std::istream& operator >> (std::istream& in, TranslationVector& t) {
  return in;
}

CalibInterface::CalibInterface(ros::NodeHandle& nh) :
        CalibrHelper(nh),
        show_surfel_map_("ui.show_surfel_map", true, false, true),
        show_all_association_points_("ui.show_all_associated_points", false, false, true),
        optimize_time_offset_("ui.optimize_time_offset", false, false, true),
        show_lidar_frame_("ui.show_lidar_frame", 1, 1, SCAN_LENGTH),
        show_p_IinL_("ui.position", TranslationVector()),
        show_q_ItoL_("ui.rpy", TranslationVector()),
        show_gravity_("ui.g", TranslationVector()),
        show_gyro_bias_("ui.GB", TranslationVector()),
        show_acce_bias_("ui.AB", TranslationVector()),
        show_time_offset_("ui.offset", 0.00, -0.1, 0.1) {

  bool show_ui = true;
  nh.param<bool>("show_ui", show_ui, true);

  if (show_ui) {
    initGui();
    pangolin::ColourWheel cw;
    for (int i = 0; i < 200; i++) {
      pangolin_colors_.emplace_back(cw.GetUniqueColour());
    }
  } else {
    Initialization();

    DataAssociation();

    BatchOptimization();

    for (size_t iter = 0; iter < 7; iter++)
      Refinement();

    opt_time_offset_ = true;
    Refinement();

    saveMap();
	std::cout << "Calibration finished." << std::endl;
  }
}


void CalibInterface::initGui() {
  pangolin::CreateWindowAndBind("Main", 1600, 1000);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  s_cam_.SetProjectionMatrix(pangolin::ProjectionMatrix(1600, 1000, 2000, 2000,
                                                        800, 500, 0.1, 1000));
  s_cam_.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 40, 0, 0, 0,
                                                      pangolin::AxisNegY));
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(UI_WIDTH));
  pointcloud_view_display_ =  &pangolin::CreateDisplay()
          .SetBounds(0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
          .SetHandler(new pangolin::Handler3D(s_cam_));

  pangolin::Var<std::function<void(void)>> initialization(
          "ui.Initialization", std::bind(&CalibInterface::Initialization, this));

  pangolin::Var<std::function<void(void)>> data_association(
          "ui.DataAssociation", std::bind(&CalibInterface::DataAssociation, this));

  pangolin::Var<std::function<void(void)>> batch_optimization(
          "ui.BatchOptimization", std::bind(&CalibInterface::BatchOptimization, this));

  pangolin::Var<std::function<void(void)>> refinement(
          "ui.Refinement", std::bind(&CalibInterface::Refinement, this));

  pangolin::Var<std::function<void(void)>> save_map(
          "ui.SaveMap", std::bind(&CalibInterface::saveMap, this));

  /// short-cut
  pangolin::RegisterKeyPressCallback('r',
                                     [this](){resetModelView();});

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'a',
                                     [&](){if(show_lidar_frame_ != 1)
                                       show_lidar_frame_ =  show_lidar_frame_ - 1;});

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'd',
                                     [&](){if(show_lidar_frame_ != SCAN_LENGTH)
                                       show_lidar_frame_ =  show_lidar_frame_ + 1;});
}


void CalibInterface::renderingLoop() {
  while (!pangolin::ShouldQuit() && ros::ok()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pointcloud_view_display_->Activate(s_cam_);
    glClearColor(0, 0, 0, 1.0f);

    if (optimize_time_offset_) {
      opt_time_offset_ = true;
    } else {
      opt_time_offset_ = false;
    }

    // show map cloud
    if (!show_surfel_map_) {
      glPointSize(1);
      glBegin(GL_POINTS);
      if (calib_step_ == RefineDone) {
        for (VPoint p: scan_undistortion_->get_map_cloud()->points) {
          glColor3f(0, 1, 1);
          glVertex3d(p.x, p.y, p.z);
        }
      } else if (calib_step_ >= DataAssociationDone) {
        for (const VPoint& p: lidar_odom_->getTargetMap()->points) {
          glColor3f(0, 1, 1);
          glVertex3d(p.x, p.y, p.z);
        }
      }
      glEnd();
    }
    else if (calib_step_ >= DataAssociationDone) {
      // show surfel map
      for (size_t i = 0; i < surfel_association_->get_surfel_planes().size(); i++) {
        glPointSize(1);
        glBegin(GL_POINTS);
        pangolin::Colour colour = pangolin_colors_[i % 200];
        for (VPoint p: surfel_association_->get_surfel_planes().at(i).cloud_inlier) {
          glColor3f(colour.red, colour.green, colour.blue);
          glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
      }
      // show scan data
      if (!show_all_association_points_) {
        double scale = (double)dataset_reader_->get_scan_data().size() / SCAN_LENGTH;
        size_t frame_id = scale * show_lidar_frame_;
        if (frame_id < dataset_reader_->get_scan_data().size()) {
          pcl::uint64_t t = dataset_reader_->get_scan_data().at(frame_id).header.stamp;
          auto iter = scan_undistortion_->get_scan_data_in_map().find(t);
          if (iter != scan_undistortion_->get_scan_data_in_map().end()) {
            glPointSize(1);
            glBegin(GL_POINTS);
            for (VPoint p: iter->second->points) {
              glColor3f(1, 1, 1);
              glVertex3d(p.x, p.y, p.z);
            }
            glEnd();
          }
        }
      }
    }

    if (show_all_association_points_) {
      glPointSize(5);
      glBegin(GL_POINTS);
      for (auto const &v : surfel_association_->get_surfel_points()) {
        glColor3f(1, 1, 1);
        glVertex3d(v.point_in_map(0), v.point_in_map(1), v.point_in_map(2));
      }
      glEnd();
    }

    if (calib_step_ >= BatchOptimizationDone) {
      showCalibResult();
    }

    pangolin::FinishFrame();
    usleep(10);   // sleep 10 ms
  }
}


void CalibInterface::showCalibResult() {
  CalibParamManager::Ptr v = traj_manager_->getCalibParamManager();

  Eigen::Vector3d euler_ItoL = (v->q_LtoI.conjugate()).toRotationMatrix().eulerAngles(0,1,2);
  euler_ItoL = euler_ItoL * 180 / M_PI;

  TranslationVector g, gb, ab, tra, q;
  tra.trans = v->q_LtoI.inverse() * (-v->p_LinI);
  g.trans = v->gravity;
  q.trans = euler_ItoL;
  gb.trans = v->gyro_bias;
  ab.trans = v->acce_bias;

  show_p_IinL_ = tra;
  show_q_ItoL_ = q;
  show_gravity_ = g;
  show_acce_bias_ = ab;
  show_gyro_bias_ = gb;
  show_time_offset_ = v->time_offset;
}