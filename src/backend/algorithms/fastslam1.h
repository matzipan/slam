//
// Created by matzipan on 11/11/16.
//

#ifndef SLAM_GUI_FASTSLAM1_H
#define SLAM_GUI_FASTSLAM1_H

#include <Eigen/Dense>

#include "Particle.h"
#include "core.h"

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
    float sigmaPhi;

    void
    predict(vector<Particle> &particles, VectorXf &xTrue, float V, float G, MatrixXf &Q, float dt);

    void update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                    vector<int> &visibleLandmarkIdentifiers, VectorXf &dataAssociationTable, MatrixXf &R);

protected:
    void predictState(Particle &particle, float V, float G, MatrixXf &Q, float dt);
    float computeWeight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    void observeHeading(Particle &particle, float phi);
};


#endif //SLAM_GUI_FASTSLAM1_H
