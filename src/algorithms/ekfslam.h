//
// Created by matzipan on 11/10/16.
//

#ifndef __EKFSLAM_H__
#define __EKFSLAM_H__

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class EKFSLAM {

public:
    EKFSLAM();

    ~EKFSLAM();

    void sim(MatrixXf &landmarks, MatrixXf &waypoints, VectorXf &x, MatrixXf &P, float Vn, float Gn, MatrixXf &Qe,
             float wheel_base, float dt, float phi, int use_heading, float sigma_phi, vector<int> ftag,
             vector<VectorXf> z, MatrixXf Re, float gate_reject, float gate_augment, bool association_known,
             bool observe, vector<VectorXf> zf, vector<int> idf, vector<VectorXf> zn,
             vector<int> data_association_table,
             bool batch_update, MatrixXf R);
};


#endif // __EKFSLAM_H__
