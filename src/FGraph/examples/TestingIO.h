//
// Created by Konstantin on 14/02/2019.
//

#ifndef MROB_TESTINGIO_H
#define MROB_TESTINGIO_H

#include <fstream>
#include <string>
#include <chrono>
#include <iostream>

#include <Eigen/Core>

#include "containers.h"

using namespace std;
using namespace std::chrono;


class TestingIO{
public:
    LoadedData read_trajectory(string path);

    void write_optimization(string path, BenchmarkResults optimization);
    void write_optimizations(string path, vector<BenchmarkResults> optimizations);
private:
    void write_optimized_trajectory(ofstream &out, BenchmarkResults optimization);
    void write_node(ofstream &out, MatX1 &state);
    void write_nodes(ofstream &out, std::vector<MatX1> states);
};

#endif //MROB_TESTINGIO_H
