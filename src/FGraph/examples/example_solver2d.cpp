//
// Created by Konstantin on 15/01/2019.
//
#include <fstream>
#include <iostream>

#include <chrono>

#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/factor1Pose2d.h"
#include "mrob/factors/factor2Odometry2d.h"
#include "mrob/factors/factor2Observation2d.h"
#include "mrob/factors/nodePose2d.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/src/SparseCore/TriangularSolver.h>
//#include <Eigen/S>

using namespace std;
using namespace std::chrono;

struct LoadedData {
    double a1, a2, a3, a4;
    vector<MatX1> true_positions;
    vector<std::shared_ptr<mrob::Node>> predictions;
    vector<Mat31> motions;
};


LoadedData load_graph(const string &file_path, bool show_output) {
    ifstream in(file_path);

    LoadedData data;

    in >> data.a1 >> data.a2 >> data.a3 >> data.a4;

    if(show_output) {
        cout << "Alphas:" << endl;
        cout << data.a1 << " " << data.a2 << " " << data.a3 << " " << data.a4 << endl;
        cout << endl << "Robot trajectory:" << endl;
    }

    int rt_count;
    in >> rt_count;

    for (int i = 0; i < rt_count; i++) {
        double x, y, theta;
        in >> x >> y >> theta;

        Mat31 pos;
        pos << x, y, theta;

        data.true_positions.push_back(pos);
    }

    if(show_output) {
        cout << endl;

        cout << "Predicted trajectory:" << endl;
    }

    int pt_count;
    in >> pt_count;

    for (int i = 0; i < pt_count; i++) {
        double x, y, theta;
        in >> x >> y >> theta;

        if(show_output) cout << x << " " << y << " " << theta << endl;

        Mat31 pos;
        pos << x, y, theta;

        data.predictions.push_back(std::shared_ptr<mrob::Node>(new mrob::NodePose2d(pos)));
    }

    if(show_output) {
        cout << endl;

        cout << "Motion commands:" << endl;
    }

    int mc_count;
    in >> mc_count;

    for (int i = 0; i < mc_count; i++) {
        double drot1, dtran, drot2;
        in >> drot1 >> dtran >> drot2;

        if(show_output) cout << drot1 << " " << dtran << " " << drot2 << endl;

        Mat31 motion;
        motion << drot1, dtran, drot2;

        data.motions.push_back(motion);
    }

    in.close();

    return data;
}

std::shared_ptr<mrob::Factor> getOdometryFactor(int i, LoadedData data) {
    Mat3 motion_cov;
    double  drot1_sq = data.motions[i][0] * data.motions[i][0],
            dtran_sq = data.motions[i][1] * data.motions[i][1],
            drot2_sq = data.motions[i][2] * data.motions[i][2];
    motion_cov <<   data.a1 * drot1_sq + data.a2 * dtran_sq, 0, 0,
            0, data.a3 * dtran_sq + data.a4 * (drot1_sq + drot2_sq), 0,
            0, 0, data.a1 * drot2_sq + data.a2 * dtran_sq;

    std::shared_ptr<mrob::Factor> factor(new mrob::Factor2Odometry2d(data.motions[i], data.predictions[i],
                                                                     data.predictions[i + 1],
                                                                     motion_cov.inverse()));
    return factor;
}

void construct(mrob::FGraphSolve &graph, LoadedData data, int from, int to) {
    for (auto i = data.predictions.begin() + from; i != data.predictions.begin() + to; ++i) {
        graph.addNode(*i);
    }

    Mat3 initialCov;
    initialCov <<   1e-12, 0 , 0,
            0, 1e-12, 0,
            0, 0, 1e-12;

    if (from == 0) {
        std::shared_ptr<mrob::Factor> anchor(new mrob::Factor1Pose2d(Mat31::Zero(), data.predictions[0], initialCov.inverse()));
        graph.addFactor(anchor);
    }
    else {
        auto factor = getOdometryFactor(from - 1, data);
        graph.addFactor(factor);
    }

    for (int i = from; i < to - 1; i++) {
        auto factor = getOdometryFactor(i, data);
        graph.addFactor(factor);
    }
}

void write_node(ofstream &out, MatX1 &state) {
    out << state[0] << " " << state[1] << " " << state[2] << endl;
}

void write_nodes(ofstream &out, std::vector<MatX1> states) {
    for (auto &i: states) write_node(out, i);
}


int main ()
{
    // You should specify full path to mrob/src/FGraph/examples/trajectory.txt
    LoadedData data = load_graph("/Users/apple/CLionProjects/mrob/src/FGraph/examples/trajectory.txt", false);

    mrob::FGraphSolve   graph_full(mrob::FGraphSolve::CHOL_ADJ,50,50),
                        graph_incremental(mrob::FGraphSolve::CHOL_ADJ,50,50);

    ofstream out("out.txt");

    high_resolution_clock::time_point t1, t2;
    vector<MatX1> fullSolvePositions, incrementalSolvePositions;

    /**
     * At first, lets try to construct A matrix and solve the problem from scratch
     * each time a new node and factor is added.
     */
    out << data.predictions.size() << endl;

    t1 = high_resolution_clock::now();
    // Build initial problem from 3 nodes and solve it
    construct(graph_full, data, 0, 3);

    graph_full.buildProblem();
    graph_full.solveOnce();

    write_nodes(out, graph_full.getEstimatedPositions());

    // Solve problem fully at each step
    for (int i = 3; i < data.predictions.size(); i++) {
        construct(graph_full, data, i, i + 1);

        graph_full.buildProblem();
        graph_full.solveOnce();

        MatX1 last = graph_full.getEstimatedPositions().back();
        write_node(out, last);
        fullSolvePositions.push_back(last);
    }

    t2 = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(t2 - t1).count();
    out << duration << endl;

    float mse = 0.0f;
    for (int i = 0; i < fullSolvePositions.size(); i++) {
        MatX1 diff = fullSolvePositions[i] - data.true_positions[i + 3];
        mse += diff.squaredNorm();
    }

    out << mse << endl;

    /**
     * Now, lest do the process incrementally, doing batch updates only 3 times
     */
    out << data.predictions.size() << endl;

    t1 = high_resolution_clock::now();
    // Build initial problem from 3 nodes and solve it
    construct(graph_incremental, data, 0, 3);

    graph_incremental.buildProblem();
    graph_incremental.solveOnce();

    write_nodes(out, graph_incremental.getEstimatedPositions());

    unsigned long diff = (data.predictions.size() - 3) / 3;
    for (int i = 0; i < 3; i++) {
        int start = 3 + diff * i, end = start + diff - 1;
        // Solve intermediate nodes incrementally
        for (int j = start; j < end; j++) {
            construct(graph_incremental, data, j, j + 1);

            graph_incremental.solveIncremental();

            MatX1 last = graph_incremental.getEstimatedPositions().back();
            write_node(out, last);
            incrementalSolvePositions.push_back(last);
        }
        // Solve principal nodes fully
        construct(graph_incremental, data, end, end + 1);

        graph_incremental.buildProblem();
        graph_incremental.solveOnce();

        MatX1 last = graph_incremental.getEstimatedPositions().back();
        write_node(out, last);
        incrementalSolvePositions.push_back(last);
    }

    t2 = high_resolution_clock::now();

    duration = duration_cast<microseconds>(t2 - t1).count();
    out << duration << endl;

    mse = 0.0f;
    for (int i = 0; i < incrementalSolvePositions.size(); i++) {
        MatX1 diff = incrementalSolvePositions[i] - data.true_positions[i + 3];
        mse += diff.squaredNorm();
    }

    out << mse << endl;

    out.close();

    return 0;
}