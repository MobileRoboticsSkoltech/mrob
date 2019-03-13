//
// Created by Konstantin on 14/02/2019.
//

#ifndef MROB_CONTAINERS_H
#define MROB_CONTAINERS_H

#include <vector>
#include <chrono>

#include "mrob/factors/factor1Pose2d.hpp"
#include "mrob/factors/factor2Odometry2d.hpp"
#include "mrob/factors/factor2Observation2d.hpp"
#include "mrob/factors/nodePose2d.hpp"

using namespace std;
using namespace std::chrono;

struct LoadedData {
    double a1, a2, a3, a4;
    vector<MatX1> true_positions;
    vector<MatX1> predictions;
    vector<Mat31> motions;
};

struct BenchmarkResults{
    vector<MatX1> online_trajectory;
    long long int duration;
    double online_mse;
};

#endif //MROB_CONTAINERS_H
