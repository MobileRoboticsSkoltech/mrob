//
// Created by Konstantin on 14/02/2019.
//

#ifndef MROB_BENCHMARK_H
#define MROB_BENCHMARK_H

#include <Eigen/LU>

#include "mrob/factor_graph_solve.hpp"

#include "containers.h"

using namespace std::chrono;

class Tester {
public:
    BenchmarkResults solve_once(LoadedData data);
    BenchmarkResults solve_full(LoadedData data);
    BenchmarkResults solve_inc(LoadedData data, vector<int> re_linearize);
    BenchmarkResults solve_inc_kf(LoadedData data, vector<int> re_linearize);
private:
    void batch_update(FGraphSolve &graph, LoadedData data, int from, int to, BenchmarkResults &bench);
    void incremental_update(FGraphSolve &graph, LoadedData data, int from, int to, BenchmarkResults &bench);

    void construct(mrob::FGraphSolve &graph, LoadedData data, int from, int to);

    std::shared_ptr<mrob::Factor> getOdometryFactor(mrob::FGraphSolve &graph, int i, LoadedData data);
    double calculate_mse(vector<MatX1> optimized_trajectory, vector<MatX1> true_trajectory);
};

#endif //MROB_BENCHMARK_H
