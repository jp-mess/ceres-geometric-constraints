#include <iostream>
#include <vector>
#include "BALProblem.h"
#include "SimpleCost.h"
#include "QuatCost.h"
#include "QuatProblem.h"
#include "Ring.h"

void PrintDoubleArrayAsVector(const double* array, int length) {
    std::cout << "[";
    for (int i = 0; i < length; ++i) {
        std::cout << array[i];
        if (i < length - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

void CheckReprojectionErrors(const BALProblem& bal_problem) {
    const double* observations = bal_problem.observations();

    double total_error = 0.0;
    int num_observations = bal_problem.num_observations();

    for (int i = 0; i < num_observations; ++i) {
        double observed_x = observations[2 * i + 0];
        double observed_y = observations[2 * i + 1];

        // Create the cost functor
        SimpleCost cost_functor(observed_x, observed_y);

        // Get camera and point for this observation
        const double* camera = bal_problem.camera_for_observation(i);
        const double* point = bal_problem.point_for_observation(i);

        //PrintDoubleArrayAsVector(camera, 9);
        //PrintDoubleArrayAsVector(point, 3);
        //std::cout << point << std::endl;
        //std::cout << observed_x << " " << observed_y << std::endl;

        // Compute the reprojection error
        double residuals[2];
        cost_functor(camera, point, residuals);
        //return;

        // Compute the squared error
        double squared_error = residuals[0] * residuals[0] + residuals[1] * residuals[1];
        total_error += squared_error;
    }

    double mean_error = total_error / num_observations;
    std::cout << "Mean Squared Reprojection Error: " << mean_error << std::endl;

    // Check if the mean error is close to zero
    // You can define your threshold for "close to zero"
    const double threshold = 1;
    if (mean_error < threshold) {
        std::cout << "Reprojection error is close to zero. The solution seems to be correct." << std::endl;
    } else {
        std::cout << "Reprojection error is not close to zero. There might be an issue with the solution." << std::endl;
    }
}

void CheckQuatReprojectionErrors(const QuatProblem& quat_problem) {
    const double* observations = quat_problem.observations();

    double total_error = 0.0;
    int num_observations = quat_problem.num_observations();

    for (int i = 0; i < num_observations; ++i) {
        double observed_x = observations[2 * i + 0];
        double observed_y = observations[2 * i + 1];

        // Create the cost functor
        QuatCost cost_functor(observed_x, observed_y);

        // Get camera (including quaternion, translation, and intrinsics) and point for this observation
        const double* extrinsics = quat_problem.extrinsic_for_observation(i);
        const double* intrinsics = quat_problem.intrinsic_for_observation(i);
        const double* point = quat_problem.point_for_observation(i);

        // Compute the reprojection error
        double residuals[2];
        cost_functor(extrinsics, intrinsics, point, residuals);

        // Compute the squared error
        double squared_error = residuals[0] * residuals[0] + residuals[1] * residuals[1];
        total_error += squared_error;
    }

    double mean_error = total_error / num_observations;
    std::cout << "Mean Squared Reprojection Error: " << mean_error << std::endl;

    // Check if the mean error is close to zero
    const double threshold = 1;
    if (mean_error < threshold) {
        std::cout << "Reprojection error is close to zero. The solution seems to be correct." << std::endl;
    } else {
        std::cout << "Reprojection error is not close to zero. There might be an issue with the solution." << std::endl;
    }
}

void CheckRingReprojectionErrors(const RingProblem& ring_problem) {
    const double* observations = ring_problem.observations();
    const double* ring_params = ring_problem.geometry_params();

    double total_error = 0.0;
    int num_observations = ring_problem.num_observations();

    for (int i = 0; i < num_observations; ++i) {
        double observed_x = observations[2 * i + 0];
        double observed_y = observations[2 * i + 1];

        // Create the cost functor
        RingCost cost_functor(observed_x, observed_y);

        // Get camera (including quaternion, theta, and intrinsics) and point for this observation
        const double* extrinsics = ring_problem.extrinsic_for_observation(i);
        const double* intrinsics = ring_problem.intrinsic_for_observation(i);
        const double* point = ring_problem.point_for_observation(i);

        // Compute the reprojection error
        double residuals[2];
        cost_functor(extrinsics, intrinsics, point, ring_params, residuals);

        // Compute the squared error
        double squared_error = residuals[0] * residuals[0] + residuals[1] * residuals[1];
        total_error += squared_error;
    }

    double mean_error = total_error / num_observations;
    std::cout << "Mean Squared Reprojection Error: " << mean_error << std::endl;

    // Check if the mean error is close to zero
    const double threshold = 1; // Adjust this threshold as necessary
    if (mean_error < threshold) {
        std::cout << "Reprojection error is close to zero. The solution seems to be correct." << std::endl;
    } else {
        std::cout << "Reprojection error is not close to zero. There might be an issue with the solution." << std::endl;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: assess_reprojection <bal_problem_file> <geometry_type> [<geometry_params_file>]\n";
        return 1;
    }

    std::string filename = argv[1];
    std::string geometryType = argv[2];

    if (geometryType == "quat") {
        if (argc != 3) {
            std::cerr << "Usage for 'quat': assess_reprojection <bal_problem_file> quat\n";
            return 1;
        }
        QuatProblem quat_problem;
        if (!quat_problem.LoadFile(filename.c_str())) {
            std::cerr << "Error loading file: " << filename << "\n";
            return 1;
        }
        CheckQuatReprojectionErrors(quat_problem);
    } else if (geometryType == "angle") {
        if (argc != 3) {
            std::cerr << "Usage for 'angle': assess_reprojection <bal_problem_file> angle\n";
            return 1;
        }
        BALProblem bal_problem;
        if (!bal_problem.LoadFile(filename.c_str())) {
            std::cerr << "Error loading file: " << filename << "\n";
            return 1;
        }
        CheckReprojectionErrors(bal_problem);
    } else if (geometryType == "ring") {
        if (argc != 4) {
            std::cerr << "Usage for 'ring': assess_reprojection <bal_problem_file> ring <geometry_params_file>\n";
            return 1;
        }
        std::string geometry_file = argv[3];
        RingProblem ring_problem;
        if (!ring_problem.LoadFiles(filename.c_str(), geometry_file.c_str())) {
            std::cerr << "Error loading file: " << filename << "\n";
            return 1;
        }
        CheckRingReprojectionErrors(ring_problem);
    } else {
        std::cerr << "Invalid geometry type: " << geometryType << "\n";
        return 1;
    }

    return 0;
}
