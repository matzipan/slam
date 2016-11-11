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

protected:

    void ekf_predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float wheel_base, float dt);

    void ekf_batch_update(VectorXf &x, MatrixXf &P, vector<VectorXf> &zf, MatrixXf &R, vector<int> &idf);

    void ekf_observe_heading(VectorXf &x, MatrixXf &P, float phi, int use_heading, float sigma_phi);

    void ekf_data_associate(VectorXf &x, MatrixXf &P, vector<VectorXf> &z, MatrixXf &R, float gate1, float gate2,
                            vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn);

    void ekf_data_associate_known(VectorXf &x, vector<VectorXf> &z, vector<int> &idz, vector<VectorXf> &zf,
                                  vector<int> &idf, vector<VectorXf> &zn, vector<int> &table);

    void ekf_augment(VectorXf &x, MatrixXf &P, vector<VectorXf> &zn, MatrixXf &Re);

    void ekf_compute_association(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &R, int idf, float &nis, float &nd);

    void ekf_observe_model(VectorXf &x, int idf, VectorXf &z, MatrixXf &H);

    void ekf_add_one_z(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &Re);
};


#endif // __EKFSLAM_H__
