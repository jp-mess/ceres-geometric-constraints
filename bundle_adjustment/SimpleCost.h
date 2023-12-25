#ifndef SIMPLECOST_H
#define SIMPLECOST_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct SimpleCost {
  SimpleCost(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {

    T translated_point[3] = {point[0] - camera[3], point[1] - camera[4], point[2] - camera[5]};

    // Inverse rotation: Rotate the point by the negative angle-axis rotation.
    T neg_angle_axis[3] = {-camera[0], -camera[1], -camera[2]};
    T p[3];
    ceres::AngleAxisRotatePoint(neg_angle_axis, translated_point, p);

    const T kEpsilon = T(1e-4);

    T xp = p[0] / (p[2] + kEpsilon);
    T yp = p[1] / (p[2] + kEpsilon);


    // Project the 3D point to the image plane
    const T& focal = camera[6];
    const T& cx = camera[7];
    const T& cy = camera[8];

    // Apply the intrinsic calibration matrix
    T predicted_x = focal * xp + cx;
    T predicted_y = focal * yp + cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;


    return true;
  }

   // Factory method to create a CostFunction object from SimpleCost.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SimpleCost, 2, 9, 3>(
        new SimpleCost(observed_x, observed_y)));
  }

  double observed_x, observed_y;
};


#endif

