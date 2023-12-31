#include "Ring.h" // Assuming this header includes the RingProblem class and necessary dependencies
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>
#include <string>


void SaveRingProblem(const RingProblem& ring_problem, const std::string& output_filename) {
    std::ofstream out_file(output_filename);
    if (!out_file) {
        std::cerr << "ERROR: unable to open output file " << output_filename << ".\n";
        return;
    }

    // Write the header: number of cameras, number of points, number of observations
    out_file << ring_problem.num_cameras() << " "
             << ring_problem.num_points() << " "
             << ring_problem.num_observations() << "\n";

    // Write the observations
    const double* observations = ring_problem.observations();
    for (int i = 0; i < ring_problem.num_observations(); ++i) {
        int camera_index = ring_problem.camera_index()[i];
        int point_index = ring_problem.point_index()[i];
        double obs_x = observations[2 * i];
        double obs_y = observations[2 * i + 1];
        out_file << camera_index << " " << point_index << " " << obs_x << " " << obs_y << "\n";
    }

    // Write the optimized camera parameters (extrinsic and intrinsic for each camera)
    for (int i = 0; i < ring_problem.num_cameras(); ++i) {
        const double* extrinsic = ring_problem.extrinsic_for_observation(i);
        double quaternion[4] = {extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]};
        double angle_axis[3];
        ceres::QuaternionToAngleAxis(quaternion, angle_axis);

        // Write angle-axis representation (3 lines)
        for (int j = 0; j < 3; ++j) {
            out_file << angle_axis[j] << "\n";
        }

        // Convert theta back to translation and write translation (3 lines)
        double theta = extrinsic[4];  // Theta is the fifth parameter
        Eigen::Vector3d translation = ThetaTo3DPoint(theta, ring_problem.ring().center(), ring_problem.ring().orientation(), ring_problem.ring().radius());
        out_file << translation.x() << "\n" << translation.y() << "\n" << translation.z() << "\n";

        // Write intrinsic parameters (3 lines)
        const double* intrinsic = ring_problem.intrinsic_for_observation(i);
        for (int j = 0; j < 3; ++j) {
            out_file << intrinsic[j] << "\n";
        }
    }

    // Write the points (each coordinate on a separate line)
    for (int i = 0; i < ring_problem.num_points(); ++i) {
        const double* point = ring_problem.points() + i * 3;
        out_file << point[0] << "\n" << point[1] << "\n" << point[2] << "\n";
    }
    
    out_file.close();
}

void SolveWithRing(const char* bal_file, const char* ring_file) {
    RingProblem ring_problem;

    // Load BAL and Ring encoding files
    if (!ring_problem.LoadFiles(bal_file, ring_file)) {
        std::cout << "Error loading BAL or Ring files: " << bal_file << std::endl;
    }

    const double* observations = ring_problem.observations();
    ceres::Problem problem;


}



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    if (argc != 4) {
        std::cerr << "Usage: bundle_adjuster <bal_problem> <geometry_params> <geometry_type>\n";
        return 1;
    }

    std::string geometryType = argv[3];

    if (geometryType == "ring") {
        SolveWithRing(argv[1], argv[2]);
    } else {
        std::cerr << "Unsupported geometry type: " << geometryType << std::endl;
        return 1;  // Return error for unsupported geometry types
    }

    return 0;
}
