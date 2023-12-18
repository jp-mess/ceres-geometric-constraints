#ifndef SIMPLECOST_H
#define SIMPLECOST_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SimpleCost {
  SimpleCost(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    const T kEpsilon = T(1e-4);

    T xp = p[0] / (p[2] + kEpsilon);
    T yp = p[1] / (p[2] + kEpsilon);


    // Project the 3D point to the image plane
    const T& focal = camera[6];
    const T& cx = camera[7];
    const T& cy = camera[8];
    
    // T xp = p[0] / p[2];
    // T yp = p[1] / p[2];

    // Apply the intrinsic calibration matrix
    T predicted_x = focal * xp + cx;
    T predicted_y = focal * yp + cy;

    // std::cout << "predicted " << predicted_x << " " << predicted_y << std::endl;
    // std::cout << "observed " << observed_x << " " << observed_y << std::endl;
    // std::cout << std::endl;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    // std::cout << residuals[0] << std::endl;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SimpleCost, 2, 9, 3>(
        new SimpleCost(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};
#endif

