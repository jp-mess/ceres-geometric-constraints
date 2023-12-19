#ifndef RING_H
#define RING_H

#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>

class RingParameterization : public ceres::LocalParameterization {
public:
    RingParameterization(const Ring& ring) : ring_(ring) {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        // Implement the logic to move on the ring
        return true;
    }

    virtual bool ComputeJacobian(const double* x, double* jacobian) const override {
        // Implement the Jacobian computation
        return true;
    }

    virtual int GlobalSize() const override { return 3; } // Size of the point on the manifold
    virtual int LocalSize() const override { return 2; } // Size of the local parameterization

private:
    Ring ring_;
};


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
