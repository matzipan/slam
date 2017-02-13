//
// Created by matzipan on 11/11/16.
//

#include <Eigen/Dense>

#include "fastslam2.h"

#include "src/backend/Particle.h"
#include "src/backend/core.h"

using namespace Eigen;

FastSLAM2::FastSLAM2() { }

FastSLAM2::~FastSLAM2() { }

void FastSLAM2::update(vector<Particle> &particles, vector<VectorXf> &zf, vector<VectorXf> &zn, vector<int> &idf,
                       vector<VectorXf> &z, VectorXf &dataAssociationTable, MatrixXf &R) {
    for (int i = 0; i < particles.size(); i++) {
        // Observe map features
        if (!zf.empty()) {
            // Sample from 'optimal' proposal distribution, then update map
            sampleProposal(particles[i], zf, idf, R);
            featureUpdate(particles[i], zf, idf, R);
        }

        // Observe new features, augment map
        if (!zn.empty()) {
            if (zf.empty()) { // Sample from proposal distribution if we have not already done so above
                VectorXf xv = multivariateGauss(particles[i].xv(), particles[i].Pv(), 1);
                particles[i].setXv(xv);
                MatrixXf pv(3, 3);
                pv.setZero();
                particles[i].setPv(pv);
            }
            addFeature(particles[i], zn, R);
        }
    }

    resampleParticles(particles, nEffective, resample);
}


void FastSLAM2::predict(vector<Particle> &particles, VectorXf &xTrue, float V, float G, MatrixXf &Q, float dt) {
    // Predict step
    for (int i = 0; i < particles.size(); i++) {
        predictState(particles[i], V, G, Q, dt);
        if (useHeading) {
            // If heading known, observe heading
            observeHeading(particles[i], xTrue(2));
        }
    }
}

/**
 *
 * @param particle
 * @param V speed
 * @param G steer angle
 * @param Q
 * @param dt
 */
void FastSLAM2::predictState(Particle &particle, float V, float G, MatrixXf &Q, float dt) {
    VectorXf xv = particle.xv();
    MatrixXf Pv = particle.Pv();

    // Jacobians
    float phi = xv(2);
    MatrixXf Gv(3, 3), Gu(3, 2);

    Gv << 1, 0, -V * dt * sin(G + phi), 0, 1, V * dt * cos(G + phi), 0, 0, 1;
    Gu << dt * cos(G + phi), -V * dt * sin(G + phi), dt * sin(G + phi), V * dt * cos(G + phi), dt * sin(G) / wheelBase, V * dt * cos(G) / wheelBase;

    // Predict covariance
    MatrixXf newPv;

    // @TODO: Pv here is somehow corrupted. Probably in sampleProposal

    // Pose predict covariance = Pose predict covariance + Control noise covariance
    newPv = Gv * Pv * Gv.transpose() + Gu * Q * Gu.transpose();
    particle.setPv(newPv);

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
    VectorXf xvTemp(3);
    xvTemp << xv(0) + V * dt * cos(G + xv(2)), xv(1) + V * dt * sin(G + xv(2)), trigonometricOffset(xv(2) + V * dt * sin(G / wheelBase));
    particle.setXv(xvTemp);
}


void FastSLAM2::observeHeading(Particle &particle, float phi) {
    float sigmaPhi = 0.01 * M_PI / 180.0;
    VectorXf xv = particle.xv();
    MatrixXf Pv = particle.Pv();

    MatrixXf H(1, 3);
    H << 0, 0, 1;

    float v = trigonometricOffset(phi - xv(2));
    josephUpdate(xv, Pv, v, pow(sigmaPhi, 2), H);

    particle.setXv(xv);
    particle.setPv(Pv);
}

float FastSLAM2::gaussEvaluate(VectorXf &v, MatrixXf &S, int logflag) {
    int D = v.size();
    MatrixXf Sc = S.llt().matrixL();

    // Normalised innovation
    VectorXf nin = Sc.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);

    int s;

    float E = 0;
    for (s = 0; s < nin.size(); s++) {
        nin(s) = pow(nin(s), 2);
        E += nin(s);
    }
    E = -0.5 * E;
    // Note: above is a fast way to compute sets of inner-products

    float C, w;
    unsigned m = min(Sc.rows(), Sc.cols());

    if (logflag != 1) {
        float prod = 1;
        for (unsigned i = 0; i < m; i++) {
            prod = prod * Sc(i, i); // Multiply the diagonals
        }
        C = pow((2 * M_PI), (D / 2)) * prod; // Normalizing term (makes Gaussian hyper volume =1)
        w = exp(E) / C; // Likelihood
    } else {
        float sum = 0;
        for (unsigned i = 0; i < m; i++) {
            sum += log(Sc(i, i));
        }
        C = 0.5 * D * log(2 * M_PI) + sum; //log of normalising term
        w = E - C; // Log-likelihood
    }
    return w;
}

/// Compute particle weight for sampling
float FastSLAM2::computeWeight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
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

    for (unsigned long i = 0; i < z.size(); i++) {
        MatrixXf S = Sf[i];
        float denoninator = 2 * M_PI * sqrt(S.determinant());
        float numerator = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w * numerator / denoninator;
    }
    return w;
}

/// Compute proposal distribution, then sample from it, and compute new particle weight
void FastSLAM2::sampleProposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    // Robot position
    VectorXf xv(particle.xv());
    // Controls (motion command)
    MatrixXf Pv(particle.Pv());

    VectorXf xv0(xv);
    MatrixXf Pv0(Pv);

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    // FIXME: z is std vector, get first item's rows?
    vector<VectorXf> zp;
    MatrixXf Hvi;
    MatrixXf Hfi;
    MatrixXf Sfi;

    // FIXME: z is std vector, get first item's rows?
    VectorXf vi(z[0].rows());

    // Process each feature, incrementally refine proposal distribution
    unsigned long i;

    for (i = 0; i < idf.size(); i++) {
        vector<int> j;
        j.push_back(idf[i]);
        zp.clear();

        computeJacobians(particle, j, R, zp, &Hv, &Hf, &Sf);

        Hvi = Hv[0];
        Hfi = Hf[0];
        Sfi = Sf[0].inverse();

        vi = z[i] - zp[0];
        vi[1] = trigonometricOffset(vi[1]);

        // Proposal covariance
        MatrixXf Pv_inv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));
        Pv = Hvi.transpose() * Sfi * Hvi + Pv_inv;
        Pv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));

        // Proposal mean
        xv = xv + Pv * Hvi.transpose() * Sfi * vi;
        particle.setXv(xv);
        particle.setPv(Pv);
    }

    // Sample from proposal distribution
    VectorXf xvs = multivariateGauss(xv, Pv, 1);
    particle.setXv(xvs);
    MatrixXf zeros(3, 3);
    zeros.setZero();
    particle.setPv(zeros);

    // Compute sample weight: w = w* p(landmarksRangeBearing|xk) p(xk|xk-1) / proposal
    VectorXf v1 = deltaXv(xv0, xvs);
    VectorXf v2 = deltaXv(xv, xvs);

    float likelihood = likelihoodGivenXv(particle, z, idf, R);
    float prior = gaussEvaluate(v1, Pv0, 0);
    float proposal = gaussEvaluate(v2, Pv, 0);

    particle.setW(particle.w() * likelihood * prior / proposal);
}

float FastSLAM2::likelihoodGivenXv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    float w = 1;
    vector<int> idfi;

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    vector<VectorXf> zp;
    VectorXf v(z[0].rows());

    for (unsigned i = 0; i < idf.size(); i++) {
        idfi.clear();
        idfi.push_back(idf[i]);
        zp.clear();

        computeJacobians(particle, idfi, R, zp, &Hv, &Hf, &Sf);

        for (unsigned k = 0; k < z[0].rows(); k++) {
            v(k) = z[i][k] - zp[0][k];
        }
        v(1) = trigonometricOffset(v(1));

        w = w * gaussEvaluate(v, Sf[0], 0);
    }
    return w;
}

VectorXf FastSLAM2::deltaXv(VectorXf &xv1, VectorXf &xv2) {
    // Compute innovation between two xv estimates, normalising the heading component
    VectorXf dx = xv1 - xv2;
    dx(2) = trigonometricOffset(dx(2));
    return dx;
}

