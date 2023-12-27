#ifndef RING_H
#define RING_H

#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>

#include <iostream> 

#include "ceres/manifold.h"
#include "ceres/rotation.h"

#include <cmath>

#include <iomanip>

void PrettyPrintArray(const double* array, int size, const std::string& prefix = "Index ") {
    for (int i = 0; i < size; ++i) {
        std::cout << prefix << i << ": " << std::fixed << std::setprecision(4) << array[i] << std::endl;
    }
}

Eigen::Vector3d ThetaTo3DPoint(double theta, 
                               const Eigen::Vector3d& center, 
                               const Eigen::Vector3d& normal, 
                               double radius) {
    // Step 1: Create a reference direction within the plane
    Eigen::Vector3d reference_direction = Eigen::Vector3d::UnitX() - (Eigen::Vector3d::UnitX().dot(normal)) * normal;
    reference_direction.normalize();

    // Compute the orthogonal direction within the plane
    Eigen::Vector3d orthogonal_direction = reference_direction.cross(normal); // bad: normal.cross(reference_direction);

    // Step 2: Calculate the point's position in the plane
    Eigen::Vector3d point_in_plane = radius * (cos(theta) * reference_direction + sin(theta) * orthogonal_direction);

    // Step 3: Translate the point back to the original coordinate system
    Eigen::Vector3d point_in_original_space = point_in_plane + center;

    return point_in_original_space;
}

double ProjectPointOntoRing(const Eigen::Vector3d& point, 
                            const Eigen::Vector3d& center, 
                            const Eigen::Vector3d& normal) {
    // Step 1: Translate the point to the ring's coordinate system
    Eigen::Vector3d translated_point = point - center;

    // Step 2: Project the translated point onto the plane defined by the ring's normal
    Eigen::Vector3d projected_point = translated_point - (translated_point.dot(normal)) * normal;

    // Step 3: Find the theta in the plane
    // Choose an arbitrary direction in the plane as the reference for theta = 0
    Eigen::Vector3d reference_direction = Eigen::Vector3d::UnitX() - (Eigen::Vector3d::UnitX().dot(normal)) * normal;
    reference_direction.normalize();

    // Compute the angle between the reference direction and the projected point
    double theta = atan2(projected_point.dot(Eigen::Vector3d::UnitZ()), projected_point.dot(reference_direction));

    return theta;
}




class Ring {
    public:
        Ring() {}
        Ring(const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, double elevation_degree)
            : center_(center), normal_(normal), radius_(radius), elevation_degree_(elevation_degree) {}

        // Getters for the ring parameters
        Eigen::Vector3d center() const { return center_; }
        Eigen::Vector3d normal() const { return normal_; }
        double radius() const { return radius_; }
        double elevation_degree() const { return elevation_degree_; }

        // TODO: Implement Plus and Jacobian operations

    private:
        Eigen::Vector3d center_;
        Eigen::Vector3d normal_;
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
    double radius;
    double elevation_degree;

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

    return Ring(center, normal, radius, elevation_degree);
}

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

    bool LoadRingFile(const char* ring_file_path) {
        try {
            ring_ = LoadRingParameters(std::string(ring_file_path));
            return true;
        } catch (const std::runtime_error& e) {
            std::cout << "Error loading ring file: " << e.what() << std::endl;
            return false;
        }
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

        num_parameters_ = n_camera_params * num_cameras_ + 3 * num_points_;
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
                parameters_[n_camera_params * i + j] = quaternion[j];  // Store quaternion
            }

            double translation[3];
            for (int j = 0; j < 3; ++j) {
                FscanfOrDie(fptr, "%lf", &translation[j]);
            }
            Eigen::Vector3d translation_vector(translation[0], translation[1], translation[2]);
            double theta = ProjectPointOntoRing(translation_vector, ring_.center(), ring_.normal());
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

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
    Ring ring_;
};

#endif
