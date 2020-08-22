/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 3/16/18.
//

#ifndef LIO_MATH_UTILS_H_
#define LIO_MATH_UTILS_H_

#pragma once

#include <cmath>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mathutils {

template<typename T>
inline T RadToDeg(T rad) {
  return rad * 180.0 / M_PI;
}

template<typename T>
inline T NormalizeRad(T rad) {
  rad = fmod(rad + M_PI, 2 * M_PI);
  if (rad < 0) {
    rad += 2 * M_PI;
  }
  return rad - M_PI;
}

template<typename T>
inline T DegToRad(T deg) {
  return deg / 180.0 * M_PI;
}

template<typename T>
inline T NormalizeDeg(T deg) {
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0) {
    deg += 360.0;
  }
  return deg - 180.0;
}

inline bool RadLt(double a, double b) {
  return NormalizeRad(a - b) < 0;
}

inline bool RadGt(double a, double b) {
  return NormalizeRad(a - b) > 0;
}

template<typename PointT>
inline PointT ScalePoint(const PointT &p, float scale) {
  PointT p_o = p;
  p_o.x *= scale;
  p_o.y *= scale;
  p_o.z *= scale;
  return p_o;
}

template<typename PointT>
inline float CalcSquaredDiff(const PointT &a, const PointT &b) {
  float diff_x = a.x - b.x;
  float diff_y = a.y - b.y;
  float diff_z = a.z - b.z;

  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcSquaredDiff(const PointT &a, const PointT &b, const float &wb) {
  float diff_x = a.x - b.x * wb;
  float diff_y = a.y - b.y * wb;
  float diff_z = a.z - b.z * wb;

  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcPointDistance(const PointT &p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template<typename PointT>
inline float CalcSquaredPointDistance(const PointT &p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

///< Matrix calculations

static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
      v3d.z(), typename Derived::Scalar(0), -v3d.x(),
      -v3d.y(), v3d.x(), typename Derived::Scalar(0);
  return m;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
  typename Derived::Scalar q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p) {
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
  typename Derived::Scalar p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> LeftQuatMatrix(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vq{q.x(), q.y(), q.z()};
  T q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> RightQuatMatrix(const Eigen::Matrix<T, 4, 1> &p) {
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vp{p.x(), p.y(), p.z()};
  T p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

// adapted from VINS-mono
inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t y = ypr(0) / 180.0 * M_PI;
  Scalar_t p = ypr(1) / 180.0 * M_PI;
  Scalar_t r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y), cos(y), 0,
      0, 0, 1;

  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p),
      0., 1., 0.,
      -sin(p), 0., cos(p);

  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0.,
      0., cos(r), -sin(r),
      0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

inline Eigen::Quaterniond g2R(Eigen::Vector3d gravity) {

  // Get z axis, which alines with -g (z_in_G=0,0,1)
  Eigen::Vector3d z_axis = -gravity/gravity.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1,0,0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
  x_axis= x_axis/x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Matrix<double,3,1> y_axis = SkewSymmetric(z_axis)*x_axis;

  // From these axes get rotation
  Eigen::Matrix<double,3,3> Ro;
  Ro.block(0,0,3,1) = x_axis;
  Ro.block(0,1,3,1) = y_axis;
  Ro.block(0,2,3,1) = z_axis;

  Eigen::Quaterniond q0(Ro);
  q0.normalize();
  return q0;
}


inline Eigen::Vector3f CrossProduct(Eigen::Vector3f a, Eigen::Vector3f b)
{
   Eigen::Vector3f c;

    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];

    return c;
}

inline float DotProduct(Eigen::Vector3f a, Eigen::Vector3f b)
{
    float result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return result;
}

inline float Normalize(Eigen::Vector3f v)
{
    float result;
    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return result;
}

inline Eigen::Matrix3f RotationMatrix(float angle, Eigen::Vector3f u)
{
    float norm = Normalize(u);
    Eigen::Matrix3f rotatinMatrix;

    u(0) = u(0) / norm;
    u(1) = u(1) / norm;
    u(2) = u(2) / norm;

    rotatinMatrix(0, 0) = cos(angle) + u(0) * u(0) * (1 - cos(angle));
    rotatinMatrix(0, 1) = u(0) * u(1) * (1 - cos(angle)) - u(2) * sin(angle);
    rotatinMatrix(0, 2) = u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));

    rotatinMatrix(1, 0) = u(2) * sin(angle) + u(0) * u(1) * (1 - cos(angle));
    rotatinMatrix(1, 1) = cos(angle) + u(1) * u(1) * (1 - cos(angle));
    rotatinMatrix(1, 2) = -u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));

    rotatinMatrix(2, 0) = -u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));
    rotatinMatrix(2, 1) = u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));
    rotatinMatrix(2, 2) = cos(angle) + u(2) * u(2) * (1 - cos(angle));

    return rotatinMatrix;
}

inline Eigen::Matrix3f Calculation(Eigen::Vector3f vectorBefore, Eigen::Vector3f vectorAfter)
{
    Eigen::Vector3f rotationAxis;
    float rotationAngle;
    Eigen::Matrix3f rotationMatrix;
    rotationAxis = CrossProduct(vectorBefore, vectorAfter);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    rotationMatrix = RotationMatrix(rotationAngle, rotationAxis);
    return rotationMatrix;
}



} // namespance mathutils

#endif //LIO_MATH_UTILS_H_
