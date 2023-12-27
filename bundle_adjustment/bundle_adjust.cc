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

#include "BALProblem.h"
#include "SimpleCost.h"
#include "QuatProblem.h"
#include "QuatCost.h"


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

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    if (argc != 2) {
        std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
        return 1;
    }

    bool useQuaternionSystem = true; // Set this to false to use the old system.

    if (useQuaternionSystem) {
        SolveWithQuaternion(argv[1]);
    } else {
        SolveWithAngleAxis(argv[1]);
    }

    return 0;
}




