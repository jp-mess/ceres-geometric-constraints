#ifndef QUATPROBLEM_H
#define QUATPROBLEM_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class QuatProblem {
 public:
  QuatProblem() : num_cameras_(0), num_points_(0), num_observations_(0), num_parameters_(0),
                  point_index_(nullptr), camera_index_(nullptr), observations_(nullptr), parameters_(nullptr) {}

  ~QuatProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations() const { return num_observations_; }
  const double* observations() const { return observations_; }
  double* mutable_cameras() { return parameters_; }
  double* mutable_points() { return parameters_ + 10 * num_cameras_; }
  int num_cameras() const { return num_cameras_; }
  int num_points() const { return num_points_; }
  const int* camera_index() const { return camera_index_; }
  const int* point_index() const { return point_index_; }
  const double* points() const {
    return parameters_ + 10 * num_cameras_;
  }

  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 10;
  }
  const double* camera_for_observation(int i) const {
    return parameters_ + camera_index_[i] * 10;
  }
  const double* point_for_observation(int i) const {
    return parameters_ + (10 * num_cameras_) + (point_index_[i] * 3);
  }

  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
}

  bool LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == nullptr) {
      return false;
    }

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 10 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }

    for (int i = 0; i < num_cameras_; ++i) {
      double angle_axis[3];
      for (int j = 0; j < 3; ++j) {
        FscanfOrDie(fptr, "%lf", &angle_axis[j]);
      }
      double quaternion[4];
      ceres::AngleAxisToQuaternion(angle_axis, quaternion);
      for (int j = 0; j < 4; ++j) {
        parameters_[10 * i + j] = quaternion[j];
      }
      for (int j = 4; j < 10; ++j) {
        FscanfOrDie(fptr, "%lf", &parameters_[10 * i + j]);
      }
    }

    for (int i = 0; i < num_points_; ++i) {
      for (int j = 0; j < 3; ++j) {
        FscanfOrDie(fptr, "%lf", &parameters_[10 * num_cameras_ + 3 * i + j]);
      }
    }

    fclose(fptr);
    return true;
  }

 private:
  template <typename T>
  void FscanfOrDie(FILE* fptr, const char* format, T* value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};
#endif