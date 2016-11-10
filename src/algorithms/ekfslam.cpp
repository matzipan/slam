//
// Created by matzipan on 11/10/16.
//

#include <Eigen/Dense>

#include "src/core.h"
#include "ekfslam.h"

using namespace std;
using namespace Eigen;

EKFSLAM::EKFSLAM() { }

EKFSLAM::~EKFSLAM() { }

void EKFSLAM::sim(MatrixXf &landmarks, MatrixXf &waypoints, VectorXf &x, MatrixXf &P, float Vn, float Gn, MatrixXf &Qe,
                  float wheel_base, float dt, float phi, int use_heading, float sigma_phi, vector<int> ftag,
                  vector<VectorXf> z, MatrixXf Re, float gate_reject, float gate_augment, bool association_known,
                  bool observe, vector<VectorXf> zf, vector<int> idf, vector<VectorXf> zn,
                  vector<int> data_association_table,
                  bool batch_update, MatrixXf R) {
    // predict position & heading
    ekf_predict(x, P, Vn, Gn, Qe, wheel_base, dt);

    // correct x and P by other sensor (low noise, IMU ...)
    ekf_observe_heading(x, P, phi, use_heading, sigma_phi);

    if (observe) {
        if (association_known) {
            ekf_data_associate_known(x, z, ftag, zf, idf, zn, data_association_table);
        } else {
            ekf_data_associate(x, P, z, Re, gate_reject, gate_augment, zf, idf, zn);
        }

        if (batch_update) {
            ekf_batch_update(x, P, zf, R, idf);
        }

        ekf_augment(x, P, zn, Re);
    }
}