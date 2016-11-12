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

    void predict(vector<Particle> &particles, VectorXf &xtrue, float V, float G, MatrixXf &Q, float wheel_base, float dt, bool add_random, bool heading_known);

    void update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                vector<VectorXf> &z, vector<int> &ftag_visible, VectorXf &data_association_table, MatrixXf &R,
                int neffective, bool do_resample);

protected:
    void predict_state(Particle &particle, float V, float G, MatrixXf &Q, float wheel_base, float dt, bool add_random);
    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
};


#endif //SLAM_GUI_FASTSLAM1_H
