#include "Ring.h" // Assuming this header includes the RingProblem class and necessary dependencies
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>

#include "Ring.h" // Assuming this header includes the RingProblem class and necessary dependencies
#include <iostream>

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
        Eigen::Vector3d translation = ThetaTo3DPoint(theta, ring_problem.ring().center(), ring_problem.ring().normal(), ring_problem.ring().radius());
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

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " BAL_file Ring_params_file" << std::endl;
        return 1;
    }

    const char* bal_file = argv[1];
    const char* ring_file = argv[2];

    RingProblem ring_problem;

    // Load ring parameters from the file
    if (!ring_problem.LoadRingFile(ring_file)) {
        std::cout << "Error loading ring parameters file: " << ring_file << std::endl;
        return 1;
    }

    // Load BAL encoding file
    if (!ring_problem.LoadFile(bal_file)) {
        std::cout << "Error loading BAL file: " << bal_file << std::endl;
        return 1;
    }

    // At this point, ring_problem is loaded with the ring parameters and the BAL encoding

    // Further processing can be done here...

    SaveRingProblem(ring_problem, "out.txt");

    return 0;
}

