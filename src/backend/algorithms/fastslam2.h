//
// Created by matzipan on 11/11/16.
//

#ifndef SLAM_GUI_FASTSLAM2_H
#define SLAM_GUI_FASTSLAM2_H

#include <Eigen/Dense>

#include "src/backend/Particle.h"
#include "src/backend/core.h"

class FastSLAM2 {

public:
    FastSLAM2();

    ~FastSLAM2();

    void
    predict(vector<Particle> &particles, VectorXf &xTrue, float V, float G, MatrixXf &Q, float dt);

    void update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                    vector<VectorXf> &z, VectorXf &dataAssociationTable, MatrixXf &R);

    bool addPredictNoise;
    bool useHeading;
    float wheelBase;
    int nEffective;
    bool resample;
    float sigmaPhi;

protected:

    VectorXf deltaXv(VectorXf &xv1, VectorXf &xv2);

    void sampleProposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    float likelihoodGivenXv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    void predictState(Particle &particle, float V, float G, MatrixXf &Q, float dt);

    void observeHeading(Particle &particle, float phi);

    float gaussEvaluate(VectorXf &v, MatrixXf &S, int logflag);

    float computeWeight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

};

#endif //SLAM_GUI_FASTSLAM2_H
