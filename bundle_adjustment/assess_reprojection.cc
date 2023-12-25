#include <iostream>
#include <vector>
#include "BALProblem.h"
#include "SimpleCost.h"
#include "QuatCost.h"
#include "QuatProblem.h"

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

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: assess_reprojection <bal_problem_file> <use_quaternion (0 or 1)>\n";
        return 1;
    }

    std::string filename = argv[1];
    bool useQuaternion = std::stoi(argv[2]);

    if (useQuaternion) {
        // Use QuatProblem and QuatCost
        QuatProblem quat_problem;
        if (!quat_problem.LoadFile(filename.c_str())) {
            std::cerr << "Error loading file: " << filename << "\n";
            return 1;
        }
        CheckQuatReprojectionErrors(quat_problem);
    } else {
        // Use BALProblem and SimpleCost
        BALProblem bal_problem;
        if (!bal_problem.LoadFile(filename.c_str())) {
            std::cerr << "Error loading file: " << filename << "\n";
            return 1;
        }
        CheckReprojectionErrors(bal_problem);
    }

    return 0;
}
