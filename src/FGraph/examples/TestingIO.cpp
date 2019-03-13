//
// Created by Konstantin on 14/02/2019.
//
#include "TestingIO.h"


LoadedData TestingIO::read_trajectory(string path) {
    ifstream in(path);

    LoadedData data;

    if (in.fail()) {
        cout << "TestingIO::read_trajectory: could not open file\n" << path;
    }

    in >> data.a1 >> data.a2 >> data.a3 >> data.a4;

    //cout << "Alphas:" << endl;
    //cout << data.a1 << " " << data.a2 << " " << data.a3 << " " << data.a4 << endl;

    int rt_count;
    in >> rt_count;

    //cout << endl;
    //cout << "Robot trajectory:" << endl;
    //cout << "Length: " << rt_count << endl;

    for (int i = 0; i < rt_count; i++) {
        double x, y, theta;
        in >> x >> y >> theta;

        Mat31 pos;
        pos << x, y, theta;

        data.true_positions.push_back(pos);
    }

    int pt_count;
    in >> pt_count;

    //cout << endl;
    //cout << "Predicted trajectory:" << endl;
    //cout << "Length: " << pt_count << endl;

    for (int i = 0; i < pt_count; i++) {
        double x, y, theta;
        in >> x >> y >> theta;

//        cout << x << " " << y << " " << theta << endl;

        Mat31 pos;
        pos << x, y, theta;

        data.predictions.push_back(pos);
    }


    int mc_count;
    in >> mc_count;

//    cout << endl;
//    cout << "Motion commands:" << endl;
//    cout << "Length: " << mc_count << endl;

    for (int i = 0; i < mc_count; i++) {
        double drot1, dtran, drot2;
        in >> drot1 >> dtran >> drot2;

//        cout << drot1 << " " << dtran << " " << drot2 << endl;

        Mat31 motion;
        motion << drot1, dtran, drot2;

        data.motions.push_back(motion);
    }

    in.close();

    return data;
}

void TestingIO::write_optimization(string path, BenchmarkResults optimization) {
    ofstream out(path);

    write_optimized_trajectory(out, optimization);

    out.close();
}

void TestingIO::write_optimizations(string path, vector<BenchmarkResults> optimizations) {
    ofstream out(path);

    for (auto &i: optimizations) write_optimized_trajectory(out, i);

    out.close();
}


void TestingIO::write_optimized_trajectory(ofstream &out, BenchmarkResults optimization) {
    out << optimization.online_trajectory.size() << endl;

    write_nodes(out, optimization.online_trajectory);

    out << optimization.duration << endl;
    out << optimization.online_mse << endl;
}

void TestingIO::write_node(ofstream &out, MatX1 &state) {
    out << state[0] << " " << state[1] << " " << state[2] << endl;
}

void TestingIO::write_nodes(ofstream &out, std::vector<MatX1> states) {
    for (auto &i: states) write_node(out, i);
}
