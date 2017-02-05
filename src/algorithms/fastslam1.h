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

    bool addPredictNoise;
    bool useHeading;
    float wheelBase;
    int nEffective;
    bool resample;

    void
    predict(vector<Particle> &particles, VectorXf &xtrue, float V, float G, MatrixXf &Q, float dt);

    void update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                    vector<int> &visibleLandmarkIdentifiers, VectorXf &dataAssociationTable, MatrixXf &R);

protected:
    void predictState(Particle &particle, float V, float G, MatrixXf &Q, float dt);
    float computeWeight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
};


#endif //SLAM_GUI_FASTSLAM1_H
