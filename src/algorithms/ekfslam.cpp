//
// Created by matzipan on 11/10/16.
//

#include <Eigen/Dense>

#include "src/core.h"
#include "ekfslam.h"

using namespace std;
using namespace Eigen;

EKFSLAM::EKFSLAM() {}

EKFSLAM::~EKFSLAM() {}

void EKFSLAM::sim(MatrixXf &landmarks, MatrixXf &waypoints, VectorXf &x, MatrixXf &P, float Vn, float Gn, MatrixXf &Qe,
                  float wheel_base, float dt, float phi, int use_heading, float sigma_phi, vector<int> ftag,
                  vector<VectorXf> z, MatrixXf Re, float gate_reject, float gate_augment, bool association_known,
                  bool observe, vector<VectorXf> zf, vector<int> idf, vector<VectorXf> zn,
                  vector<int> data_association_table,
                  bool doBatchUpdate, MatrixXf R) {
    // Predict position & heading
    predict(x, P, Vn, Gn, Qe, wheel_base, dt);

    // Correct xEstimated and P by other sensor (low noise, IMU ...)
    observeHeading(x, P, phi, use_heading, sigma_phi);

    if (observe) {
        if (association_known) {
            dataAssociateKnown(x, z, ftag, zf, idf, zn, data_association_table);
        } else {
            dataAssociate(x, P, z, Re, gate_reject, gate_augment, zf, idf, zn);
        }

        if (doBatchUpdate) {
            batchUpdate(x, P, zf, R, idf);
        }

        augment(x, P, zn, Re);
    }
}


void EKFSLAM::predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float wheel_base, float dt) {
    float s, c;
    float vts, vtc;
    MatrixXf Gv(3, 3), Gu(3, 2);         // Jacobians

    s = sin(G + x(2));
    c = cos(G + x(2));
    vts = V * dt * s;
    vtc = V * dt * c;

    Gv << 1, 0, -vts,
            0, 1, vtc,
            0, 0, 1;
    Gu << dt * c, -vts,
            dt * s, vtc,
            dt * sin(G) / wheel_base, V * dt * cos(G) / wheel_base;

    // predict covariance
    //      P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
    P.block(0, 0, 3, 3) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();

    int m = P.rows();
    if (m > 3) {
        P.block(0, 3, 3, m - 3) = Gv * P.block(0, 3, 3, m - 3);
        P.block(3, 0, m - 3, 3) = P.block(0, 3, 3, m - 3).transpose();
    }

    // predict state
    x(0) = x(0) + vtc;
    x(1) = x(1) + vts;
    x(2) = trigonometricOffset(x(2) + V * dt * sin(G) / wheel_base);
}

void EKFSLAM::observeHeading(VectorXf &x, MatrixXf &P, float phi, int use_heading, float sigma_phi) {
    if (use_heading == 0) { return; }

    float v;
    MatrixXf H = MatrixXf(1, x.size());

    H.setZero(1, x.size());
    H(2) = 1;
    v = trigonometricOffset(phi - x(2));

    KF_joseph_update(x, P, v, sigma_phi * sigma_phi, H);
}

void EKFSLAM::ekf_observe_model(VectorXf &x, int idf, VectorXf &z, MatrixXf &H) {
    int Nxv, fpos;
    float dx, dy, d2, d, xd, yd, xd2, yd2;

    Nxv = 3;
    H.setZero(2, x.size());
    z.setZero(2);

    // position of xf in state
    fpos = Nxv + idf * 2;

    dx = x(fpos) - x(0);
    dy = x(fpos + 1) - x(1);
    d2 = dx * dx + dy * dy;
    d = sqrtf(d2);
    xd = dx / d;
    yd = dy / d;
    xd2 = dx / d2;
    yd2 = dy / d2;

    z(0) = d;
    z(1) = atan2(dy, dx) - x(2);

    H(0, 0) = -xd;
    H(0, 1) = -yd;
    H(0, 2) = 0;
    H(1, 0) = yd2;
    H(1, 1) = -xd2;
    H(1, 2) = -1;

    H(0, fpos) = xd;
    H(0, fpos + 1) = yd;
    H(1, fpos) = -yd2;
    H(1, fpos + 1) = xd2;
}

void
EKFSLAM::ekf_compute_association(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &R, int idf, float &nis, float &nd) {
    VectorXf zp;
    MatrixXf H;
    VectorXf v(2);
    MatrixXf S;

    ekf_observe_model(x, idf, zp, H);

    v = z - zp;
    v(1) = trigonometricOffset(v(1));

    S = H * P * H.transpose() + R;

    nis = v.transpose() * S.inverse() * v;
    nd = nis + log(S.determinant());
}

void EKFSLAM::dataAssociate(VectorXf &x, MatrixXf &P, vector<VectorXf> &z, MatrixXf &R, float gate1, float gate2,
                            vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn) {
    unsigned long Nxv, Nf;
    unsigned long i, j;
    long jbest;
    float nis, nd;
    float nbest, outer;

    zf.clear();
    zn.clear();
    idf.clear();

    Nxv = 3;                        // number of vehicle pose states
    Nf = (x.size() - Nxv) / 2;       // number of features already in map

    // linear search for nearest-neighbour, no clever tricks (like a quick
    // bounding-box threshold to remove distant features; or, better yet,
    // a balanced k-d tree lookup). TODO: implement clever tricks.
    for (i = 0; i < z.size(); i++) {
        jbest = -1;
        nbest = 1e60;
        outer = 1e60;

        for (j = 0; j < Nf; j++) {
            ekf_compute_association(x, P, z[i], R, j, nis, nd);

            if (nis < gate1 && nd < nbest) {
                nbest = nd;
                jbest = j;
            } else if (nis < outer) {
                outer = nis;
            }
        }

        if (jbest > -1) {
            // add nearest-neighbour to association list
            zf.push_back(z[i]);
            idf.push_back(jbest);
        } else if (outer > gate2) {
            // landmarksRangeBearing too far to associate, but far enough to be a new feature
            zn.push_back(z[i]);
        }
    }
}

//landmarksRangeBearing is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void EKFSLAM::dataAssociateKnown(VectorXf &x, vector<VectorXf> &z, vector<int> &idz, vector<VectorXf> &zf,
                                 vector<int> &idf, vector<VectorXf> &zn, vector<int> &table) {
    vector<int> idn;
    unsigned i, ii;

    zf.clear();
    zn.clear();
    idf.clear();
    idn.clear();

    for (i = 0; i < idz.size(); i++) {
        ii = idz[i];
        VectorXf z_i;

        if (table[ii] == -1) {
            // new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            // exist feature
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(table[ii]);
        }
    }

    // add new feature IDs to lookup table
    int Nxv = 3;                        // number of vehicle pose states
    int Nf = (x.size() - Nxv) / 2;       // number of features already in map

    // add new feature positions to lookup table
    //      table[idn]= Nf + (1:size(zn,2));
    for (i = 0; i < idn.size(); i++) {
        table[idn[i]] = Nf + i;
    }
}

void EKFSLAM::batchUpdate(VectorXf &x, MatrixXf &P, vector<VectorXf> &zf, MatrixXf &R, vector<int> &idf) {
    int lenz, lenx;
    MatrixXf H, RR;
    VectorXf v;
    VectorXf zp;
    MatrixXf H_;

    int i;

    lenz = zf.size();
    lenx = x.size();

    H.setZero(2 * lenz, lenx);
    v.setZero(2 * lenz);
    RR.setZero(2 * lenz, 2 * lenz);

    zp.setZero(2);

    for (i = 0; i < lenz; i++) {
        ekf_observe_model(x, idf[i], zp, H_);
        H.block(i * 2, 0, 2, lenx) = H_;

        v(2 * i) = zf[i](0) - zp(0);
        v(2 * i + 1) = trigonometricOffset(zf[i](1) - zp(1));

        RR.block(i * 2, i * 2, 2, 2) = R;
    }

    KF_cholesky_update(x, P, v, RR, H);
}

void EKFSLAM::ekf_add_one_z(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &Re) {
    int len;
    float r, b;
    float s, c;

    MatrixXf Gv, Gz;

    len = x.size();

    r = z(0);
    b = z(1);
    s = sin(x(2) + b);
    c = cos(x(2) + b);

    // augment xEstimated
    //      xEstimated = [xEstimated; xEstimated(1)+r*c; xEstimated(2)+r*s];
    x.conservativeResize(len + 2);
    x(len) = x(0) + r * c;
    x(len + 1) = x(1) + r * s;

    // Jacobians
    Gv.setZero(2, 3);
    Gz.setZero(2, 2);

    Gv << 1, 0, -r * s,
            0, 1, r * c;
    Gz << c, -r * s,
            s, r * c;

    // augment P
    P.conservativeResize(len + 2, len + 2);

    // feature cov
    //      P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz';
    P.block(len, len, 2, 2) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() +
                              Gz * Re * Gz.transpose();

    // vehicle to feature xcorr
    //      P(rng,1:3)= Gv*P(1:3,1:3);
    //      P(1:3,rng)= P(rng,1:3)';
    P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
    P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();

    if (len > 3) {
        // map to feature xcoor
        //   P(rng,rnm)= Gv*P(1:3,rnm);
        P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);

        //   P(rnm,rng)= P(rng,rnm)';
        P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
    }
}

void EKFSLAM::augment(VectorXf &x, MatrixXf &P, vector<VectorXf> &zn, MatrixXf &Re) {
    for (unsigned long i = 0; i < zn.size(); i++) {
        ekf_add_one_z(x, P, zn[i], Re);
    }
}