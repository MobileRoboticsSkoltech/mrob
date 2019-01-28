//
// Created by Konstantin on 15/01/2019.
//
#include <fstream>
#include <iostream>

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

struct LoadedData {
    double a1, a2, a3, a4;
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

        if(show_output) cout << x << " " << y << " " << theta << endl;
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

void save_graph(mrob::FGraphSolve &graph) {
    auto nodes = graph.getNodes();

    ofstream out("out.txt");

    out << nodes.size() << endl;

    for (auto &i: nodes) {
        out << i->getState()[0] << " " << i->getState()[1] << " " << i->getState()[2] << endl;
    }

    out.close();
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


int main ()
{
    // You should specify full path to mrob/src/FGraph/examples/trajectory.txt
    LoadedData data = load_graph("/Users/apple/CLionProjects/mrob/src/FGraph/examples/trajectory.txt", false);

    mrob::FGraphSolve   graph_full(mrob::FGraphSolve::CHOL_ADJ,50,50),
                        graph_incremental(mrob::FGraphSolve::CHOL_ADJ,50,50);

    construct(graph_full, data, 0, 4);
    graph_full.buildProblem();
    graph_full.solveOnce();

    cout << endl;

    construct(graph_incremental, data, 0, 3);
    graph_incremental.buildProblem();
    graph_incremental.solveOnce();

    cout << endl;

    construct(graph_incremental, data, 3, 4);
    graph_incremental.solveIncremental();

    return 0;
}



//    Mat2 matrix;
//    matrix <<   4,-2,
//                -2, 10;
//
//    Mat21 rhs;
//    rhs << 4, 10;
//
//    Eigen::LLT<Mat2> solver(matrix);
//    auto dx_1 = solver.solve(rhs);
//    cout << dx_1 << endl;
//
//    auto y_1 = solver.matrixL().solve(rhs);
//    auto dx_2 = solver.matrixU().solve(y_1);
//    cout << dx_2 << endl;
//
//    auto R = solver.matrixU().toDenseMatrix();
//    auto L = solver.matrixL().toDenseMatrix();
//
//    Eigen::TriangularView<Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Lower> solver_2(L);
//    auto y_2 = solver_2.solve(rhs);
//    Eigen::TriangularView<Eigen::Matrix<double, 2, 2, 1, 2, 2>, Eigen::Upper> solver_3(R);
//    auto dx_3 = solver_3.solve(y_2);
//    cout << dx_3 << endl;

//    Mat31 z1, z2;
//    z1 << -0.089545, 0.03355436, 0.01867014;
//    z2 << -0.0464949, 0.07584124, -0.50885916;
//
//    Mat3 obsCov;
//    obsCov <<   1e-12, 0, 0,
//            0, 1e-12, 0,
//            0, 0, 1e-12;
//
//    std::shared_ptr<mrob::Factor>   obs_factor1(new mrob::Factor2Observation2d(z1, data.predictions[2],
//            data.predictions[14], obsCov.inverse())),
//            obs_factor2(new mrob::Factor2Observation2d(z2, data.predictions[3], data.predictions[15], obsCov.inverse()));
//
//    graph.addFactor(obs_factor1);
//    graph.addFactor(obs_factor2);
