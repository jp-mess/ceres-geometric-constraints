// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2023 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/manifold.h"
#include "ceres/product_manifold.h"
#include "ceres/autodiff_manifold.h"

#include "BALProblem.h"
#include "SimpleCost.h"
#include "QuatProblem.h"
#include "QuatCost.h"
#include "Ring.h"
#include "angle_manifold.h"

void SaveRingProblem(const RingProblem& ring_problem, const std::string& output_filename, const std::string& ring_params_filename) {
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


    // Save the estimated ring parameters
    std::ofstream ring_file(ring_params_filename);
    if (!ring_file) {
        std::cerr << "ERROR: unable to open ring parameters file " << ring_params_filename << ".\n";
        return;
    }

    const double* geometry_params = ring_problem.geometry_params();
    Eigen::Vector3d center(geometry_params[0], geometry_params[1], geometry_params[2]);

    // Eigen is (w,x,y,z) while Ceres is (x,y,z,w)
    Eigen::Quaterniond quaternion(geometry_params[6], geometry_params[3], geometry_params[4], geometry_params[5]);

    double radius = geometry_params[7];

    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d estimated_normal = quaternion * z_axis;

    // Write ring parameters
    ring_file << "type: ring\n";
    ring_file << "center: " << center.x() << "," << center.y() << "," << center.z() << "\n";
    ring_file << "normal: " << estimated_normal.x() << "," << estimated_normal.y() << "," << estimated_normal.z() << "\n";
    ring_file << "radius: " << radius << "\n";
    ring_file << "elevation_degree: NaN\n";  // Placeholder for elevation degree

    ring_file.close();
    
}

void SaveBALProblem(const BALProblem& bal_problem, const std::string& output_filename) {
    std::ofstream out_file(output_filename);
    if (!out_file) {
        std::cerr << "ERROR: unable to open output file " << output_filename << ".\n";
        return;
    }

    // Write the header: number of cameras, number of points, number of observations
    out_file << bal_problem.num_cameras() << " "
             << bal_problem.num_points() << " "
             << bal_problem.num_observations() << "\n";

    // Write the observations
    const double* observations = bal_problem.observations();
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        int camera_index = bal_problem.camera_index()[i];
        int point_index = bal_problem.point_index()[i];
        double obs_x = observations[2 * i];
        double obs_y = observations[2 * i + 1];
        out_file << camera_index << " " << point_index << " " << obs_x << " " << obs_y << "\n";
    }

    // Write the optimized camera parameters
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        const double* camera = bal_problem.camera_for_observation(i);
        for (int j = 0; j < 9; ++j) {
            out_file << camera[j] << "\n";
        }
    }

    for (int i = 0; i < bal_problem.num_points(); ++i) {
        const double* point = bal_problem.points() + i * 3;  // Access each point using the new const method
        for (int j = 0; j < 3; ++j) {
            out_file << point[j] << "\n";
        }
    }
    

    out_file.close();
}

void SaveQuatProblem(const QuatProblem& quat_problem, const std::string& output_filename) {
    std::ofstream out_file(output_filename);
    if (!out_file) {
        std::cerr << "ERROR: unable to open output file " << output_filename << ".\n";
        return;
    }

    // Write the header: number of cameras, number of points, number of observations
    out_file << quat_problem.num_cameras() << " "
             << quat_problem.num_points() << " "
             << quat_problem.num_observations() << "\n";

    // Write the observations
    const double* observations = quat_problem.observations();
    for (int i = 0; i < quat_problem.num_observations(); ++i) {
        int camera_index = quat_problem.camera_index()[i];
        int point_index = quat_problem.point_index()[i];
        double obs_x = observations[2 * i];
        double obs_y = observations[2 * i + 1];
        out_file << camera_index << " " << point_index << " " << obs_x << " " << obs_y << "\n";
    }

    // Write the optimized camera parameters (extrinsic and intrinsic for each camera)
    for (int i = 0; i < quat_problem.num_cameras(); ++i) {
        const double* extrinsic = quat_problem.extrinsic_for_observation(i);
        double quaternion[4] = {extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]};
        double angle_axis[3];
        ceres::QuaternionToAngleAxis(quaternion, angle_axis);

        for (int j = 0; j < 3; ++j) {
            out_file << angle_axis[j] << "\n";
        }
        for (int j = 3; j < 7; ++j) {  // Translation part
            out_file << extrinsic[j] << "\n";
        }

        const double* intrinsic = quat_problem.intrinsic_for_observation(i);
        for (int j = 0; j < 3; ++j) {
            out_file << intrinsic[j] << "\n";
        }
    }

    // Write the points
    for (int i = 0; i < quat_problem.num_points(); ++i) {
        const double* point = quat_problem.points() + i * 3;
        for (int j = 0; j < 3; ++j) {
            out_file << point[j] << "\n";
        }
    }
    
    out_file.close();
}

void SolveWithQuaternion(const char* filename) {
    QuatProblem quat_problem;
    if (!quat_problem.LoadFile(filename)) {
      std::cerr << "ERROR: unable to open file " << filename << "\n";
      exit(1);
    }

    const double* observations = quat_problem.observations();
    ceres::Problem problem;

    for (int i = 0; i < quat_problem.num_observations(); ++i) {
        ceres::CostFunction* cost_function = QuatCost::Create(
            observations[2 * i + 0], observations[2 * i + 1]);

        // Get the entire camera parameter block (which includes quaternion, translation, and intrinsics)
        double* extrinsics = quat_problem.mutable_extrinsic_for_observation(i);
        double* intrinsics = quat_problem.mutable_intrinsic_for_observation(i);
        double* point = quat_problem.mutable_point_for_observation(i);


        ceres::Manifold* camera_manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{};

        problem.AddParameterBlock(intrinsics, 3);
        problem.SetParameterBlockConstant(intrinsics);

        problem.AddParameterBlock(extrinsics, 7, camera_manifold); // assuming camera has 7 parameters (4 for quaternion, 3 for translation)

        // Add the residual block to the problem.
        problem.AddResidualBlock(cost_function, nullptr /* squared loss */, extrinsics, intrinsics, point);

}


    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    SaveQuatProblem(quat_problem, "test.txt");
}

void SolveWithAngleAxis(const char* filename) {
    BALProblem bal_problem;
    if (!bal_problem.LoadFile(filename)) {
      std::cerr << "ERROR: unable to open file " << filename << "\n";
      exit(1);
    }

    const double* observations = bal_problem.observations();
    ceres::Problem problem;

    for (int i = 0; i < bal_problem.num_observations(); ++i) {
      ceres::CostFunction* cost_function = SimpleCost::Create(
          observations[2 * i + 0], observations[2 * i + 1]);

      double* camera = bal_problem.mutable_camera_for_observation(i);
      double* point = bal_problem.mutable_point_for_observation(i);

      problem.AddResidualBlock(cost_function, nullptr, camera, point);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

}

void SolveWithRing(const char* filename, const char* ring_params_filename) {
    RingProblem ring_problem;
    if (!ring_problem.LoadFiles(filename, ring_params_filename) || !ring_problem.LoadRingFile(ring_params_filename)) {
      std::cerr << "ERROR: unable to open file " << filename << "\n";
      exit(1);
    }

    const double* observations = ring_problem.observations();
    ceres::Problem problem;

    for (int i = 0; i < ring_problem.num_observations(); ++i) {
        ceres::CostFunction* cost_function = RingCost::Create(
            observations[2 * i + 0], observations[2 * i + 1]);

        // Get camera parameters and point for this observation
        double* extrinsics = ring_problem.mutable_extrinsic_for_observation(i);
        double* intrinsics = ring_problem.mutable_intrinsic_for_observation(i);
        double* point = ring_problem.mutable_point_for_observation(i);
        double* geometry = ring_problem.mutable_geometry_params();

        // Use a product manifold of AngleManifold (for theta) and QuaternionManifold (for the quaternion)
        ceres::Manifold* camera_manifold = new ceres::ProductManifold<ceres::AutoDiffManifold<AngleManifold, 1, 1>, ceres::QuaternionManifold>{};

        problem.AddParameterBlock(intrinsics, 3);
        problem.SetParameterBlockConstant(intrinsics);

        ceres::Manifold* ring_manifold = new ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::QuaternionManifold, ceres::EuclideanManifold<1>>{};

        problem.AddParameterBlock(geometry, 8, ring_manifold);

        // Assuming camera has 5 parameters (1 for theta, 4 for quaternion)
        // problem.AddParameterBlock(extrinsics, 5, camera_manifold);
        problem.AddParameterBlock(extrinsics, 5);

        // Add the residual block to the problem
        problem.AddResidualBlock(cost_function, nullptr /* squared loss */, extrinsics, intrinsics, point, geometry);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    SaveRingProblem(ring_problem, "../../problem_encodings/outputs/ring_solution.txt", "../../geometry_encodings/outputs/ring_estimated.txt");
}

/**
 * Main function for running the bundle adjuster.
 * 
 * This program adjusts the camera parameters and 3D world points to minimize
 * reprojection error, based on the provided BAL (Bundle Adjustment in the Large) problem file.
 * It supports different geometry types: quaternion, angle-axis, and ring.
 * 
 * Usage:
 * 
 * For quaternion-based optimization:
 *   bundle_adjuster <bal_problem_file> quat
 *   - <bal_problem_file>: Path to the BAL problem file.
 *   - 'quat': Specifies that quaternion-based optimization is to be used.
 * 
 * For angle-axis-based optimization:
 *   bundle_adjuster <bal_problem_file> angle
 *   - <bal_problem_file>: Path to the BAL problem file.
 *   - 'angle': Specifies that angle-axis-based optimization is to be used.
 * 
 * For ring-based optimization:
 *   bundle_adjuster <bal_problem_file> ring <geometry_params_file>
 *   - <bal_problem_file>: Path to the BAL problem file.
 *   - 'ring': Specifies that ring-based optimization is to be used.
 *   - <geometry_params_file>: Path to the file containing the geometry parameters for the ring.
 * 
 * The program will choose the optimization method based on the specified geometry type
 * and perform bundle adjustment accordingly.
 */

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: bundle_adjuster <bal_problem_file> <geometry_type> [<geometry_params_file>]\n";
        return 1;
    }

    std::string filename = argv[1];
    std::string geometryType = argv[2];

    if (geometryType == "quat") {
        if (argc != 3) {
            std::cerr << "Usage for 'quat': bundle_adjuster <bal_problem_file> quat\n";
            return 1;
        }
        SolveWithQuaternion(filename.c_str());
    } else if (geometryType == "angle") {
        if (argc != 3) {
            std::cerr << "Usage for 'angle': bundle_adjuster <bal_problem_file> angle\n";
            return 1;
        }
        SolveWithAngleAxis(filename.c_str());
    } else if (geometryType == "ring") {
        if (argc != 4) {
            std::cerr << "Usage for 'ring': bundle_adjuster <bal_problem_file> ring <geometry_params_file>\n";
            return 1;
        }
        std::string ring_params_file = argv[3];
        SolveWithRing(filename.c_str(), ring_params_file.c_str());
    } else {
        std::cerr << "Invalid geometry type: " << geometryType << "\n";
        return 1;
    }

    return 0;
}




