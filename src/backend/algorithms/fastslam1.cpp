//
// Created by matzipan on 11/11/16.
//

#include <Eigen/Dense>

#include "fastslam1.h"

#include "src/backend/Particle.h"
#include "src/backend/core.h"

using namespace Eigen;

FastSLAM1::FastSLAM1() { }

FastSLAM1::~FastSLAM1() { }

void FastSLAM1::update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                       vector<int> &visibleLandmarkIdentifiers, VectorXf &dataAssociationTable, MatrixXf &R) {
    // Perform update
    for (int i = 0; i < particles.size(); i++) {
        if (!zf.empty()) {
            // Observe map features
            float w = computeWeight(particles[i], zf, idf, R);
            w = particles[i].w() * w;
            particles[i].setW(w);
            featureUpdate(particles[i], zf, idf, R);
        }
        if (!zn.empty()) {
            addFeature(particles[i], zn, R);
        }
    }

    resampleParticles(particles, nEffective, resample);
}

void FastSLAM1::predictState(Particle &particle, float V, float G, MatrixXf &Q, float dt) {
    // Optional: add random noise to predicted state
    if (addPredictNoise) {
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
    VectorXf xvTemp(3);
    xvTemp << xv(0) + V * dt * cos(G + xv(2)), xv(1) + V * dt * sin(G + xv(2)), trigonometricOffset(xv(2) + V * dt * sin(G / wheelBase));
    particle.setXv(xvTemp);
}

// Predict step
void FastSLAM1::predict(vector<Particle> &particles, VectorXf &xTrue, float V, float G, MatrixXf &Q, float dt) {
    for (int i = 0; i < particles.size(); i++) {
        predictState(particles[i], V, G, Q, dt);

        if (useHeading) {
            // If heading known, observe heading
            observeHeading(particles[i], xTrue(2));
        }
    }
}

/**
 * Correct xEstimated and P by other sensor (low noise, IMU ...)
 *
 * @param particle
 * @param phi
 */
void FastSLAM1::observeHeading(Particle &particle, float phi) {
    VectorXf xv = particle.xv();
    MatrixXf Pv = particle.Pv();

    MatrixXf H(1, 3);
    H << 0, 0, 1;

    float v = trigonometricOffset(phi - xv(2));
    josephUpdate(xv, Pv, v, pow(sigmaPhi, 2), H);

    particle.setXv(xv);
    particle.setPv(Pv);
}



// Compute particle weight for sampling
float FastSLAM1::computeWeight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    // Process each feature, incrementally refine proposal distribution
    computeJacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    vector<VectorXf> v;

    for (unsigned long j = 0; j < z.size(); j++) {
        VectorXf v_j = z[j] - zp[j];
        v_j[1] = trigonometricOffset(v_j[1]);
        v.push_back(v_j);
    }

    float w = 1.0;

    MatrixXf S;
    for (unsigned long i = 0; i < z.size(); i++) {
        S = Sf[i];
        float den = 2 * M_PI * sqrt(S.determinant());
        float num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w * num / den;
    }
    return w;
}

