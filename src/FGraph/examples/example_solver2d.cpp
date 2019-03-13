//
// Created by Konstantin on 15/01/2019.
//
#include "TestingIO.h"
#include "Tester.h"

int main ()
{
    TestingIO testingIO;
    //string  source = "/Users/apple/PycharmProjects/mrob_bench/main/out";
    string  source = "/home/gonzalo/mrob/mrob/src/FGraph/examples/";
    //string  destination = "/Users/apple/PycharmProjects/mrob_bench/main/optimized";
    string  destination = "/home/gonzalo/mrob/mrob/src/FGraph/examples/";

    //vector<string> trajectories = {"line_trajectory.txt", "parabola_trajectory.txt", "sin_trajectory.txt"};
    vector<string> trajectories = {"trajectory.txt"};

    vector<vector<int>> re_linearization_points = {
            vector<int>(),
            vector<int>(),
            vector<int>()
    };

    for (int i = 0; i < 18; i+=3) {
        re_linearization_points[0].push_back(i);
        re_linearization_points[1].push_back(i);
        re_linearization_points[2].push_back(i);
    }

    Tester tester;

    vector<LoadedData> data;

    for (auto i: trajectories) data.push_back(testingIO.read_trajectory(source + "/" + i));

    for (uint_t j = 0; j < data.size(); j++) {
        auto bench_once = tester.solve_once(data[j]);
        testingIO.write_optimization(destination + "/once_" + trajectories[j], bench_once);
    }

    for (uint_t j = 0; j < data.size(); j++) {
        auto bench_full = tester.solve_full(data[j]);
        testingIO.write_optimization(destination + "/full_" + trajectories[j], bench_full);
    }

    /* Incremental function not working properly TODO check
    for (uint_t j = 0; j < data.size(); j++) {
        auto bench_inc = tester.solve_inc(data[j], re_linearization_points[j]);
        testingIO.write_optimization(destination + "/inc_" + trajectories[j], bench_inc);
    }*/

    return 0;
}
