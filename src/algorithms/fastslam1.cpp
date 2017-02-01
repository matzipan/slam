//
// Created by matzipan on 11/11/16.
//

#include <Eigen/Dense>

#include "fastslam1.h"

#include "src/particle.h"
#include "src/core.h"

using namespace Eigen;

FastSLAM1::FastSLAM1() { }

FastSLAM1::~FastSLAM1() { }

void FastSLAM1::update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                       vector<VectorXf> &z, vector<int> &ftag_visible, VectorXf &data_association_table, MatrixXf &R,
                       int neffective, bool do_resample) {
    // Perform update
    for (int i = 0; i < particles.size(); i++) {
        if (!zf.empty()) { //observe map features
            float w = compute_weight(particles[i], zf, idf, R);
            w = particles[i].w() * w;
            particles[i].setW(w);
            feature_update(particles[i], zf, idf, R);
        }
        if (!zn.empty()) {
            add_feature(particles[i], zn, R);
        }
    }

    resample_particles(particles, neffective, do_resample);
}

void FastSLAM1::predict_state(Particle &particle, float V, float G, MatrixXf &Q, float wheel_base, float dt, bool add_random) {
    // Optional: add random noise to predicted state
    if (add_random) {
        VectorXf A(2);
        A(0) = V;
        A(1) = G;
        VectorXf VG(2);
        VG = multivariateGauss(A, Q, 1);
        V = VG(0);
        G = VG(1);
    }

    // Predict state
    VectorXf xv = particle.xv();
    VectorXf xv_temp(3);
    xv_temp << xv(0) + V * dt * cos(G + xv(2)), xv(1) + V * dt * sin(G + xv(2)), trigonometricOffset(xv(2) + V * dt * sin(G / wheel_base));
    particle.setXv(xv_temp);
}

// Predict step
void FastSLAM1::predict(vector<Particle> &particles, VectorXf &xtrue, float V, float G, MatrixXf &Q, float wheel_base, float dt, bool add_random, bool heading_known) {
    for (int i = 0; i < particles.size(); i++) {
        predict_state(particles[i], V, G, Q, wheel_base, dt, add_random);

        if (heading_known) { // If heading known, observe heading
            for (unsigned long j = 0; j < particles[i].xf().size(); j++) {
                VectorXf xf_j = particles[i].xf()[j];
                xf_j[2] = xtrue[2];
                particles[i].setXfi(j, xf_j);
            }
        }
    }
}



// Compute particle weight for sampling
float FastSLAM1::compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    // Process each feature, incrementally refine proposal distribution
    compute_jacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    vector<VectorXf> v;

    for (unsigned long j = 0; j < z.size(); j++) {
        VectorXf v_j = z[j] - zp[j];
        v_j[1] = trigonometricOffset(v_j[1]);
        v.push_back(v_j);
    }

    float w = 1.0;

    MatrixXf S;
    float den, num;
    for (unsigned long i = 0; i < z.size(); i++) {
        S = Sf[i];
        den = 2 * M_PI * sqrt(S.determinant());
        num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w * num / den;
    }
    return w;
}

