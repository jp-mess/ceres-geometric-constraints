#ifndef QUATCOST_H
#define QUATCOST_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct QuatCost {
  QuatCost(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const extrinsic_params, // Camera parameters
                  const T* const intrinsic_params, // frozen intrinsics
                  const T* const point,  // 3D point
                  T* residuals) const {

    // Camera parameters: quaternion (4), translation (3), intrinsics (3)
    const T* quaternion = extrinsic_params;
    const T* translation = extrinsic_params + 4;
    const T* intrinsics = intrinsic_params;

    // Conjugate of the quaternion for inverse rotation.
    T conjugate_quaternion[4] = {quaternion[0], 
                                 -quaternion[1], 
                                 -quaternion[2], 
                                 -quaternion[3]};

    // Apply inverse translation: point - translation.
    T translated_point[3] = {point[0] - translation[0],
                             point[1] - translation[1],
                             point[2] - translation[2]};


    // Rotate the translated point using the conjugate of the camera quaternion.
    T rotated_translated_point[3];
    ceres::QuaternionRotatePoint(conjugate_quaternion, translated_point, rotated_translated_point);

    // Project the 3D point onto the 2D camera plane.
    const T& focal = intrinsics[0];
    const T& cx = intrinsics[1];
    const T& cy = intrinsics[2];

    const T kEpsilon = T(1e-4);
    const T xp = rotated_translated_point[0] / (rotated_translated_point[2] + kEpsilon);
    const T yp = rotated_translated_point[1] / (rotated_translated_point[2] + kEpsilon);

    const T predicted_x = focal * xp + cx;
    const T predicted_y = focal * yp + cy;

    // The error is the difference between the predicted and observed positions.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    // 2: residuals (pixels)
    // 10: camera parameter block (3 translation, 4 quaternion, 3 intrinsics)
    // 3: world point parameter block
    return new ceres::AutoDiffCostFunction<QuatCost, 2, 7, 3, 3>(
        new QuatCost(observed_x, observed_y));
  }

  double observed_x, observed_y;
};
#endif
