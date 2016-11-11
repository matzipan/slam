//
// Created by matzipan on 11/11/16.
//

#ifndef SLAM_GUI_FASTSLAM1_H
#define SLAM_GUI_FASTSLAM1_H

#include <Eigen/Dense>

#include "src/particle.h"
#include "src/core.h"

using namespace Eigen;

class FastSLAM1 {

public:
    FastSLAM1();

    ~FastSLAM1();

    void sim (vector<Particle> particles, vector<VectorXf> &z, vector<int> &ftag_vibisle, VectorXf &data_association_table, MatrixXf &R, int neffective, bool do_resample);

protected:
    void predict(Particle &particle, float V, float G, MatrixXf &Q, float WB, float dt, int addrandom);
    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
};


#endif //SLAM_GUI_FASTSLAM1_H
