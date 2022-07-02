#include "benchmark.h"

#include "problem_generator.h"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace poselib {

template <typename Solver> BenchmarkResult benchmark(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<AbsolutePoseProblemInstance> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        // std::cout << "\nGt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        // std::cout << "gt valid = " << Solver::validator::is_valid(instance, instance.pose_gt, 1.0, tol) << "\n";
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, 1.0, tol))
                result.valid_solutions_++;

//            std::cout << "Pose: " << pose.R() << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose, 1.0));
        }

        std::cout << pose_error << std::endl;

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());
    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Solver>
BenchmarkResult benchmark_w_extra(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<AbsolutePoseProblemInstance> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;
        std::vector<double> extra;

        int sols = Solver::solve(instance, &solutions, &extra);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        for (size_t k = 0; k < solutions.size(); ++k) {
            if (Solver::validator::is_valid(instance, solutions[k], extra[k], tol))
                result.valid_solutions_++;
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, solutions[k], extra[k]));
        }

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    std::vector<double> extra;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance &instance : problem_instances) {
            solutions.clear();
            extra.clear();

            int sols = Solver::solve(instance, &solutions, &extra);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());
    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Solver>
BenchmarkResult benchmark_relative(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<RelativePoseProblemInstance> problem_instances;
    generate_relpose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const RelativePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        // std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, tol))
                result.valid_solutions_++;
            // std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
        }

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const RelativePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());

    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Solver>
BenchmarkResult benchmark_homography(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<RelativePoseProblemInstance> problem_instances;
    generate_homography_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const RelativePoseProblemInstance &instance : problem_instances) {
        std::vector<Eigen::Matrix3d> solutions;

        int sols = Solver::solve(instance, &solutions);

        double hom_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        // std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        for (const Eigen::Matrix3d &H : solutions) {
            if (Solver::validator::is_valid(instance, H, tol))
                result.valid_solutions_++;
            // std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            hom_error = std::min(hom_error, Solver::validator::compute_pose_error(instance, H));
        }

        if (hom_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    std::vector<Eigen::Matrix3d> solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const RelativePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());

    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

} // namespace poselib

void print_runtime(double runtime_ns) {
    if (runtime_ns < 1e3) {
        std::cout << runtime_ns << " ns";
    } else if (runtime_ns < 1e6) {
        std::cout << runtime_ns / 1e3 << " us";
    } else if (runtime_ns < 1e9) {
        std::cout << runtime_ns / 1e6 << " ms";
    } else {
        std::cout << runtime_ns / 1e9 << " s";
    }
}

void display_result(const std::vector<poselib::BenchmarkResult> &results) {
    // Print PoseLib version and buidling type
    std::cout << "\n" << poselib_info() << "\n\n";

    int w = 13;
    // display header
    std::cout << std::setw(2 * w) << "Solver";
    std::cout << std::setw(w) << "Solutions";
    std::cout << std::setw(w) << "Valid";
    std::cout << std::setw(w) << "GT found";
    std::cout << std::setw(w) << "Runtime"
              << "\n";
    for (int i = 0; i < w * 6; ++i)
        std::cout << "-";
    std::cout << "\n";

    int prec = 6;

    for (const poselib::BenchmarkResult &result : results) {
        double num_tests = static_cast<double>(result.instances_);
        double solutions = result.solutions_ / num_tests;
        double valid_sols = result.valid_solutions_ / static_cast<double>(result.solutions_) * 100.0;
        double gt_found = result.found_gt_pose_ / num_tests * 100.0;
        double runtime_ns = result.runtime_ns_ / num_tests;

        std::cout << std::setprecision(prec) << std::setw(2 * w) << result.name_;
        std::cout << std::setprecision(prec) << std::setw(w) << solutions;
        std::cout << std::setprecision(prec) << std::setw(w) << valid_sols;
        std::cout << std::setprecision(prec) << std::setw(w) << gt_found;
        std::cout << std::setprecision(prec) << std::setw(w - 3);
        print_runtime(runtime_ns);
        std::cout << "\n";
    }
}

int main() {

    std::vector<poselib::BenchmarkResult> results;

    poselib::ProblemOptions options;
    // options.camera_fov_ = 45; // Narrow
    options.camera_fov_ = 75; // Medium
    // options.camera_fov_ = 120; // Wide

    double tol = 2;

    // P3P
//    poselib::ProblemOptions p3p_opt = options;
//    p3p_opt.n_point_point_ = 3;
//    p3p_opt.n_point_line_ = 0;
//    results.push_back(poselib::benchmark<poselib::SolverP3P>(1e5, p3p_opt, tol));

    // uP2P
    poselib::ProblemOptions up2p_opt = options;
    up2p_opt.n_point_point_ = 2;
    up2p_opt.n_point_line_ = 0;
    up2p_opt.upright_ = true;
    up2p_opt.dev_ = 2;
    results.push_back(poselib::benchmark<poselib::SolverUP2P>(10, up2p_opt, tol));

    display_result(results);

    return 0;
}
