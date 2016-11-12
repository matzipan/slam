//
// Created by matzipan on 11/11/16.
//

#ifndef SLAM_GUI_FASTSLAM2_H
#define SLAM_GUI_FASTSLAM2_H

#include <Eigen/Dense>

#include "src/particle.h"
#include "src/core.h"

class FastSLAM2 {

public:
    FastSLAM2();

    ~FastSLAM2();

    void predict(vector<Particle> &particles, VectorXf &xtrue, float V, float G, MatrixXf &Q, float wheel_base, float dt, bool add_random, bool heading_known);

    void update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                vector<VectorXf> &z, vector<int> &ftag_visible, VectorXf &data_association_table, MatrixXf &R,
                int neffective, bool do_resample);
protected:

    void sample_proposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    float likelihood_given_xv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    VectorXf delta_xv(VectorXf &xv1, VectorXf &xv2);

    void predict_state(Particle &particle, float V, float G, MatrixXf &Q, float wheel_base, float dt, int add_random);

    void observe_heading(Particle &particle, float phi);

    float gauss_evaluate(VectorXf &v, MatrixXf &S, int logflag);

    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    vector<Particle> sim(MatrixXf &lm, MatrixXf &wp);
};

#endif //SLAM_GUI_FASTSLAM2_H
