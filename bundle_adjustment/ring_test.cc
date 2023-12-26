#include "BALProblem.h"
#include "Ring.h"
#include <iostream>
#include <Eigen/Dense>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " BAL_file Ring_params_file" << std::endl;
        return 1;
    }

    const std::string bal_file = argv[1];
    const std::string ring_file = argv[2];

    // Load BAL data
    BALProblem bal_problem;
    if (!bal_problem.LoadFile(bal_file.c_str())) {
        std::cerr << "Error loading BAL file." << std::endl;
        return 1;
    }

    // Load ring parameters
    Ring ring = LoadRingParameters(ring_file);

    // Iterate over all cameras
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        const double* camera = bal_problem.mutable_cameras() + 9 * i; // Assuming 10 parameters per camera
        Eigen::Vector3d translation(camera[3], camera[4], camera[5]); // Assuming translation parameters

        double theta = ProjectPointOntoRing(translation, ring.center(), ring.normal());
        Eigen::Vector3d reconstructed_point = ThetaTo3DPoint(theta, ring.center(), ring.normal(), ring.radius());

        double residual = (translation - reconstructed_point).norm();

        std::cout << "Camera " << i << ":" << std::endl;
        std::cout << "  Original Point: " << translation.transpose() << std::endl;
        std::cout << "  Reconstructed Point: " << reconstructed_point.transpose() << std::endl;
        std::cout << "  Residual Distance: " << residual << std::endl;
    }

    return 0;
}
