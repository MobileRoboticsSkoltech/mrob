//
// Created by Konstantin on 14/02/2019.
//
#include "Tester.h"

BenchmarkResults Tester::solve_once(LoadedData data) {
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ, 50, 50);

    BenchmarkResults bench;
    high_resolution_clock::time_point t1, t2;

    t1 = high_resolution_clock::now();

    batch_update(graph, data, 0, data.predictions.size(), bench);

    t2 = high_resolution_clock::now();

    bench.duration = duration_cast<microseconds>(t2 - t1).count();
    bench.online_mse = calculate_mse(bench.online_trajectory, data.true_positions);

    return bench;
}

BenchmarkResults Tester::solve_full(LoadedData data) {
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ, 50, 50);

    BenchmarkResults bench;
    high_resolution_clock::time_point t1, t2;

    t1 = high_resolution_clock::now();

    batch_update(graph, data, 0, 3, bench);

    for (uint_t i = 3; i < data.predictions.size(); i++) batch_update(graph, data, i, i + 1, bench);

    t2 = high_resolution_clock::now();

    bench.duration = duration_cast<microseconds>(t2 - t1).count();
    bench.online_mse = calculate_mse(bench.online_trajectory, data.true_positions);

    return bench;
}

BenchmarkResults Tester::solve_inc(LoadedData data, vector<int> re_linearize) {
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ, 50, 50);

    BenchmarkResults bench;
    high_resolution_clock::time_point t1, t2;

    t1 = high_resolution_clock::now();

    batch_update(graph, data, 0, 3, bench);

    int i = 3;
    for (uint_t j = 0; j < re_linearize.size(); j++) {
        for (; i < re_linearize[j] - 1; i++) {
          incremental_update(graph, data, i, i + 1, bench);
        }
        batch_update(graph, data, i, i + 1, bench);
        ++i;
    }

    for (; i < (int)data.predictions.size() - 1; i++) incremental_update(graph, data, i, i + 1, bench);

    batch_update(graph, data, i, i + 1, bench);

    t2 = high_resolution_clock::now();

    bench.duration = duration_cast<microseconds>(t2 - t1).count();
    bench.online_mse = calculate_mse(bench.online_trajectory, data.true_positions);

    return bench;
}


void Tester::batch_update(FGraphSolve &graph, LoadedData data, int from, int to, BenchmarkResults &bench) {
    construct(graph, data, from, to);

    graph.buildProblem();
    graph.solveOnce();

    auto poses = graph.getEstimatedPositions();
    bench.online_trajectory.insert(bench.online_trajectory.end(), poses.end() - (to - from), poses.end());
}

void Tester::incremental_update(FGraphSolve &graph, LoadedData data, int from, int to, BenchmarkResults &bench) {
    construct(graph, data, from, to);

    graph.solveIncremental();

    auto poses = graph.getEstimatedPositions();
    bench.online_trajectory.insert(bench.online_trajectory.end(), poses.end() - (to - from), poses.end());
}


void Tester::construct(mrob::FGraphSolve &graph, LoadedData data, int from, int to) {
    for (auto i = data.predictions.begin() + from; i != data.predictions.begin() + to; ++i) {
        std::shared_ptr<mrob::Node> node(new mrob::NodePose2d(MatX1(*i)));
        graph.addNode(node);
    }

    Mat3 initialCov;
    initialCov <<   1e-12, 0 , 0,
            0, 1e-12, 0,
            0, 0, 1e-12;

    if (from == 0) {
        std::shared_ptr<mrob::Factor> anchor(new mrob::Factor1Pose2d(Mat31::Zero(), graph.getNode(0), initialCov.inverse()));
        graph.addFactor(anchor);
    }
    else {
        auto factor = getOdometryFactor(graph, from - 1, data);
        graph.addFactor(factor);
    }

    for (int i = from; i < to - 1; i++) {
        auto factor = getOdometryFactor(graph, i, data);
        graph.addFactor(factor);
    }
}


std::shared_ptr<mrob::Factor> Tester::getOdometryFactor(mrob::FGraphSolve &graph, int i, LoadedData data) {
    Mat3 motion_cov;
    double  drot1_sq = data.motions[i][0] * data.motions[i][0],
            dtran_sq = data.motions[i][1] * data.motions[i][1],
            drot2_sq = data.motions[i][2] * data.motions[i][2];
    motion_cov <<   data.a1 * drot1_sq + data.a2 * dtran_sq, 0, 0,
            0, data.a3 * dtran_sq + data.a4 * (drot1_sq + drot2_sq), 0,
            0, 0, data.a1 * drot2_sq + data.a2 * dtran_sq;

    std::shared_ptr<mrob::Factor> factor(new mrob::Factor2Odometry2d(data.motions[i],
            graph.getNode(i),
            graph.getNode(i + 1),
            motion_cov.inverse()));
    return factor;
}

double Tester::calculate_mse(vector<MatX1> optimized_trajectory, vector<MatX1> true_trajectory) {
    double mse = 0.0f;

    for (uint_t i = 0; i < optimized_trajectory.size(); i++) {
        MatX1 diff = optimized_trajectory[i] - true_trajectory[i];
        mse += diff.squaredNorm();
    }

    return mse;
}
