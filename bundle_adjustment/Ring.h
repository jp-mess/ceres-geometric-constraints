#ifndef RING_H
#define RING_H

#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>

#include <iostream> 

#include "ceres/manifold.h"
#include "ceres/rotation.h"
#include "ceres/ceres.h"

#include <cmath>

#include <iomanip>

void PrettyPrintArray(const double* array, int size, const std::string& prefix = "Index ") {
    for (int i = 0; i < size; ++i) {
        std::cout << prefix << i << ": " << std::fixed << std::setprecision(4) << array[i] << std::endl;
    }
}


Eigen::Vector3d ThetaTo3DPoint(double theta, 
                               const Eigen::Vector3d& center, 
                               const Eigen::Quaterniond& ring_orientation, 
                               double radius) {
    // Step 1: Calculate the point's position in the ring's local XY plane
    Eigen::Vector3d point_in_plane(radius * cos(theta), radius * sin(theta), 0.0);

    // Step 2: Rotate the point from the ring's local coordinate system to the global coordinate system
    Eigen::Vector3d point_in_global_space = ring_orientation * point_in_plane;

    // Step 3: Translate the point by the ring's center to get its position in the original coordinate system
    Eigen::Vector3d point_in_original_space = point_in_global_space + center;

    return point_in_original_space;
}

template <typename T>
Eigen::Matrix<T, 3, 1> ThetaTo3DPoint(const T& theta, 
                                      const Eigen::Matrix<T, 3, 1>& center, 
                                      const Eigen::Quaternion<T>& ring_orientation, 
                                      const T& radius) {
    // Step 1: Calculate the point's position in the ring's local XY plane
    Eigen::Matrix<T, 3, 1> point_in_plane(radius * cos(theta), radius * sin(theta), T(0));

    // Step 2: Rotate the point from the ring's local coordinate system to the global coordinate system
    Eigen::Matrix<T, 3, 1> point_in_global_space = ring_orientation * point_in_plane;

    // Step 3: Translate the point by the ring's center to get its position in the original coordinate system
    Eigen::Matrix<T, 3, 1> point_in_original_space = point_in_global_space + center;

    return point_in_original_space;
}


double ProjectPointOntoRing(const Eigen::Vector3d& point, 
                            const Eigen::Vector3d& center, 
                            const Eigen::Quaterniond& ring_orientation) {
    // Step 1: Translate the point to the ring's coordinate system
    Eigen::Vector3d translated_point = point - center;

    // Step 2: Rotate the translated point to align with the ring's local coordinate system
    // Inverse rotation is used to bring the point into the ring's coordinate frame
    Eigen::Quaterniond inverse_orientation = ring_orientation.conjugate();
    Eigen::Vector3d aligned_point = inverse_orientation * translated_point;

    // Step 3: Since the ring lies in the XY plane of its local coordinate system,
    // the Z-component of aligned_point can be ignored for theta calculation.
    // Compute the angle between the X-axis and the projected point in the XY plane
    double theta = atan2(aligned_point.y(), aligned_point.x());

    return theta;
}


class Ring {
public:

    Ring() : center_(Eigen::Vector3d::Zero()), 
             orientation_(Eigen::Quaterniond::Identity()), 
             radius_(0.0), 
             elevation_degree_(0.0) {}

    Ring(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double radius, double elevation_degree)
        : center_(center), orientation_(orientation), radius_(radius), elevation_degree_(elevation_degree) {
        // Normalize the quaternion to ensure it's a valid rotation
        orientation_.normalize();
    }

    // Getters for the ring parameters
    Eigen::Vector3d center() const { return center_; }
    Eigen::Quaterniond orientation() const { return orientation_; }
    double radius() const { return radius_; }
    double elevation_degree() const { return elevation_degree_; }

private:
    Eigen::Vector3d center_;
    Eigen::Quaterniond orientation_;  // Represents the ring's orientation in 3D space
    double radius_;
    double elevation_degree_;
};



Ring LoadRingParameters(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + file_path);
    }

    std::string line;
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    double radius = 0.0;
    double elevation_degree = 0.0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, ':')) {
            if (key == "center") {
                char comma;
                iss >> center[0] >> comma >> center[1] >> comma >> center[2];
            } else if (key == "normal") {
                char comma;
                iss >> normal[0] >> comma >> normal[1] >> comma >> normal[2];
            } else if (key == "radius") {
                iss >> radius;
            } else if (key == "elevation_degree") {
                iss >> elevation_degree;
            }
        }
    }

    // Convert the normal vector to a quaternion
    Eigen::Quaterniond orientation;
    Eigen::Vector3d z_axis(0, 0, 1);
    if (normal.norm() > std::numeric_limits<double>::epsilon()) {
        normal.normalize();
        orientation = Eigen::Quaterniond::FromTwoVectors(z_axis, normal);
    } else {
        // Default orientation if normal is zero or nearly zero
        orientation = Eigen::Quaterniond::Identity();
    }

    return Ring(center, orientation, radius, elevation_degree);
}
struct RingCost {
    RingCost(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const extrinsic_params, // Camera parameters
                    const T* const intrinsic_params, // frozen intrinsics
                    const T* const point,  // 3D point
                    const T* const ring_params, // Ring parameters
                    T* residuals) const {

        // Camera parameters: quaternion (4), theta (1), intrinsics (3)
        const T* camera_quaternion = extrinsic_params;
        const T& theta = extrinsic_params[4];  // Theta is the fifth parameter
        const T* intrinsics = intrinsic_params;

        // Conjugate of the camera quaternion for inverse rotation.
        T conjugate_camera_quaternion[4] = {camera_quaternion[0], 
                                            -camera_quaternion[1], 
                                            -camera_quaternion[2], 
                                            -camera_quaternion[3]};
 
        // Extract the ring parameters: center (3), orientation quaternion (4), radius (1)
        Eigen::Matrix<T, 3, 1> center;
        Eigen::Quaternion<T> ring_orientation;
        center << ring_params[0], ring_params[1], ring_params[2];
        ring_orientation.coeffs() << ring_params[3], ring_params[4], ring_params[5], ring_params[6];
        const T& radius = ring_params[7];

        // Convert theta back to translation
        Eigen::Matrix<T, 3, 1> translation = ThetaTo3DPoint(theta, center, ring_orientation, radius);

        // Apply inverse translation: point - translation.
        T translated_point[3] = {point[0] - translation[0],
                                point[1] - translation[1],
                                point[2] - translation[2]};

        // Rotate the translated point using the conjugate of the camera quaternion.
        T rotated_translated_point[3];
        ceres::QuaternionRotatePoint(conjugate_camera_quaternion, translated_point, rotated_translated_point);

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
        // Adjust the size of the parameter blocks for RingCost
        // 5: camera parameter block (4 quaternion, 1 theta)
        // 3: intrinsic parameter block
        // 3: world point parameter block
        // 8: ring parameter block (3 center, 4 quaternion, 1 radius)
        return new ceres::AutoDiffCostFunction<RingCost, 2, 5, 3, 3, 8>(
            new RingCost(observed_x, observed_y));
    }

    double observed_x, observed_y;
};


class RingProblem {
public:
    int n_extrinsic = 5;
    int n_intrinsic = 3;
    int n_camera_params = n_extrinsic + n_intrinsic;

    RingProblem() {}

    ~RingProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int num_observations() const { return num_observations_; }
    const double* observations() const { return observations_; }
    double* mutable_cameras() { return parameters_; }
    double* mutable_points() { return parameters_ + n_camera_params * num_cameras_; }
    int num_cameras() const { return num_cameras_; }
    int num_points() const { return num_points_; }
    const int* camera_index() const { return camera_index_; }
    const int* point_index() const { return point_index_; }
    const double* points() const { return parameters_ + n_camera_params * num_cameras_; }
    const double* point_for_observation(int i) const { return parameters_ + (n_camera_params * num_cameras_) + (point_index_[i] * 3); }
    double* mutable_point_for_observation(int i) { return mutable_points() + point_index_[i] * 3; }
    double* mutable_extrinsic_for_observation(int i) { return mutable_cameras() + camera_index_[i] * n_camera_params; }
    double* mutable_intrinsic_for_observation(int i) { return mutable_cameras() + camera_index_[i] * n_camera_params + n_extrinsic; }
    const double* extrinsic_for_observation(int i) const { return parameters_ + camera_index_[i] * n_camera_params; }
    const double* intrinsic_for_observation(int i) const { return parameters_ + camera_index_[i] * n_camera_params + n_extrinsic; }
    const Ring& ring() const { return ring_;}
    double* mutable_geometry_params() {
        return parameters_ + ring_params_start_index;
    }

    
    const double* geometry_params() const {
        return parameters_ + ring_params_start_index;
    }


    bool LoadRingFile(const char* ring_file_path) {
        try {
            ring_ = LoadRingParameters(std::string(ring_file_path));
            return true;
        } catch (const std::runtime_error& e) {
            std::cout << "Error loading ring file: " << e.what() << std::endl;
            return false;
        }
    }

   bool LoadFiles(const char* bal_file, const char* ring_file_path) {
    LoadRingFile(ring_file_path);

    FILE* fptr = fopen(bal_file, "r");
    if (fptr == nullptr) {
        return false;
    }

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    int n_geometry_params = 3 + 4 + 1; // Center + Quaternion + Radius
    int n_result_params = n_camera_params * num_cameras_ + 3 * num_points_;
    num_parameters_ = n_result_params + n_geometry_params;
    parameters_ = new double[num_parameters_];

    // Index where the ring parameters start
    ring_params_start_index = n_result_params;

    // Store the ring's center coordinates
    parameters_[ring_params_start_index] = ring_.center().x();
    parameters_[ring_params_start_index + 1] = ring_.center().y();
    parameters_[ring_params_start_index + 2] = ring_.center().z();

    // Store the ring's orientation quaternion
    Eigen::Quaterniond ring_orientation = ring_.orientation();
    parameters_[ring_params_start_index + 3] = ring_orientation.x();
    parameters_[ring_params_start_index + 4] = ring_orientation.y();
    parameters_[ring_params_start_index + 5] = ring_orientation.z();
    parameters_[ring_params_start_index + 6] = ring_orientation.w();

    // Store the ring's radius
    parameters_[ring_params_start_index + 7] = ring_.radius();

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
            parameters_[n_camera_params * i + j] = quaternion[j];
        }

        double translation[3];
        for (int j = 0; j < 3; ++j) {
            FscanfOrDie(fptr, "%lf", &translation[j]);
        }
        Eigen::Vector3d translation_vector(translation[0], translation[1], translation[2]);

        // Use the updated ProjectPointOntoRing function
        Eigen::Quaterniond ring_orientation(ring_.orientation().w(), ring_.orientation().x(), ring_.orientation().y(), ring_.orientation().z());
        double theta = ProjectPointOntoRing(translation_vector, ring_.center(), ring_orientation);
        parameters_[n_camera_params * i + 4] = theta;  // Store theta

        // Store intrinsic parameters
        for (int j = 0; j < 3; j++) {
            FscanfOrDie(fptr, "%lf", &parameters_[n_camera_params*i + n_extrinsic + j]);
        }
    }

    // World Points
    for (int i = 0; i < num_points_; ++i) {
        for (int j = 0; j < 3; ++j) {
            FscanfOrDie(fptr, "%lf", &parameters_[n_camera_params * num_cameras_ + 3 * i + j]);
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
    int ring_params_start_index;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
    Ring ring_;
};

#endif
