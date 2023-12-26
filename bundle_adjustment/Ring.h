#ifndef RING_H
#define RING_H

#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>

#include "ceres/manifold.h"

#include <cmath>


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

/*
class RingManifold : public ceres::Manifold {
public:
    RingManifold(const Ring& ring) : ring_(ring) {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        // Implement the logic to move on the ring
        return true;
    }

    //virtual bool ComputeJacobian(const double* x, double* jacobian) const override {
        // Implement the Jacobian computation
    //    return true;
    //}

    virtual int AmbientSize() const override { return 3; } // Size of the space in which the manifold is embedded
    virtual int TangentSize() const override { return 2; } // Size of a point on the manifold

private:
    Ring ring_;
};
*/


class Ring {
public:
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



#endif
