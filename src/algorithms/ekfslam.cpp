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

void EKFSLAM::sim(MatrixXf &landmarks, MatrixXf &waypoints, VectorXf &x, MatrixXf &P, float noisyV, float noisyG,
                  MatrixXf &Qe, float dt, float phi, float sigmaPhi, vector<int> landmarkIdentifiers, vector<VectorXf> landmarksRangeBearing,
                  MatrixXf Re, bool observe, vector<VectorXf> zf, vector<int> idf, vector<VectorXf> zn,
                  vector<int> dataAssociationTable, MatrixXf R) {
    // Predict position & heading
    predict(x, P, noisyV, noisyG, Qe, wheelBase, dt);

    // Correct xEstimated and P by other sensor (low noise, IMU ...)
    if (useHeading) {
        observeHeading(x, P, phi, sigmaPhi);
    }

    if (observe) {
        if (associationKnown) {
            dataAssociateKnown(x, landmarksRangeBearing, landmarkIdentifiers, zf, idf, zn, dataAssociationTable);
        } else {
            dataAssociate(x, P, landmarksRangeBearing, Re, gateReject, gateAugment, zf, idf, zn);
        }

        if (enableBatchUpdate) {
            batchUpdate(x, P, zf, R, idf);
        }

        augment(x, P, zn, Re);
    }
}


void EKFSLAM::predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float wheelBase, float dt) {
    float s, c;
    float vts, vtc;
    // Jacobians
    MatrixXf Gv(3, 3), Gu(3, 2);

    s = sin(G + x(2));
    c = cos(G + x(2));
    vts = V * dt * s;
    vtc = V * dt * c;

    Gv << 1, 0, -vts,
            0, 1, vtc,
            0, 0, 1;
    Gu << dt * c, -vts,
            dt * s, vtc,
            dt * sin(G) / wheelBase, V * dt * cos(G) / wheelBase;

    // Predict covariance
    P.block(0, 0, 3, 3) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();

    int m = P.rows();
    if (m > 3) {
        P.block(0, 3, 3, m - 3) = Gv * P.block(0, 3, 3, m - 3);
        P.block(3, 0, m - 3, 3) = P.block(0, 3, 3, m - 3).transpose();
    }

    // Predict state
    x(0) = x(0) + vtc;
    x(1) = x(1) + vts;
    x(2) = trigonometricOffset(x(2) + V * dt * sin(G) / wheelBase);
}

/**
 * Correct xEstimated and P by other sensor (low noise, IMU ...)
 *
 * @param x
 * @param P
 * @param phi
 * @param sigmaPhi heading uncertainty
 */
void EKFSLAM::observeHeading(VectorXf &x, MatrixXf &P, float phi, float sigmaPhi) {
    float v;
    MatrixXf H = MatrixXf(1, x.size());

    H.setZero(1, x.size());
    H(2) = 1;
    v = trigonometricOffset(phi - x(2));

    josephUpdate(x, P, v, pow(sigmaPhi, 2), H);
}

void EKFSLAM::ekfObserveModel(VectorXf &x, int idf, VectorXf &z, MatrixXf &H) {
    int Nxv, fpos;
    float dx, dy, d2, d, xd, yd, xd2, yd2;

    Nxv = 3;
    H.setZero(2, x.size());
    z.setZero(2);

    // position of landmarkXs in state
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
EKFSLAM::ekfComputeAssociation(VectorXf &x, MatrixXf &P, VectorXf &landmarksRangeBearing, MatrixXf &R, int idf, float &nis, float &nd) {
    VectorXf zp;
    MatrixXf H;
    VectorXf v(2);
    MatrixXf S;

    ekfObserveModel(x, idf, zp, H);

    v = landmarksRangeBearing - zp;
    v(1) = trigonometricOffset(v(1));

    S = H * P * H.transpose() + R;

    nis = v.transpose() * S.inverse() * v;
    nd = nis + log(S.determinant());
}

void EKFSLAM::dataAssociate(VectorXf &x, MatrixXf &P, vector<VectorXf> &landmarksRangeBearing, MatrixXf &R, float gate1, float gate2,
                            vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn) {
    float nis, nd;

    zf.clear();
    zn.clear();
    idf.clear();

    unsigned long Nxv = 3;                        // number of vehicle pose states
    unsigned long Nf = (x.size() - Nxv) / 2;       // number of features already in map

    // Linear search for nearest-neighbour, no clever tricks (like a quick bounding-box threshold to remove distant
    // features; or, better yet, a balanced k-d tree lookup). TODO: implement clever tricks.
    for (unsigned long i = 0; i < landmarksRangeBearing.size(); i++) {
        long jbest = -1;
        float nbest = 1e60;
        float outer = 1e60;

        for (unsigned long j = 0; j < Nf; j++) {
            ekfComputeAssociation(x, P, landmarksRangeBearing[i], R, j, nis, nd);

            if (nis < gate1 && nd < nbest) {
                nbest = nd;
                jbest = j;
            } else if (nis < outer) {
                outer = nis;
            }
        }

        if (jbest > -1) {
            // add nearest-neighbour to association list
            zf.push_back(landmarksRangeBearing[i]);
            idf.push_back(jbest);
        } else if (outer > gate2) {
            // landmarksRangeBearing too far to associate, but far enough to be a new feature
            zn.push_back(landmarksRangeBearing[i]);
        }
    }
}

/**
 * Find associations zf and new features zn
 * @param x
 * @param landmarkRangeBearing is range and bearing of visible landmarks
 * @param idz
 * @param zf
 * @param idf
 * @param zn
 * @param dataAssociationTable
 */
void EKFSLAM::dataAssociateKnown(VectorXf &x, vector<VectorXf> &landmarksRangeBearing, vector<int> &idz, vector<VectorXf> &zf,
                                 vector<int> &idf, vector<VectorXf> &zn, vector<int> &dataAssociationTable) {
    vector<int> idn;

    zf.clear();
    zn.clear();
    idf.clear();

    for (unsigned int i = 0; i < idz.size(); i++) {
        unsigned int ii = idz[i];
        VectorXf z_i;

        if (dataAssociationTable[ii] == -1) {
            // new feature
            z_i = landmarksRangeBearing[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            // exist feature
            z_i = landmarksRangeBearing[i];
            zf.push_back(z_i);
            idf.push_back(dataAssociationTable[ii]);
        }
    }

    // Add new feature IDs to lookup dataAssociationTable
    // Number of vehicle pose states
    int Nxv = 3;
    // Number of features already in map
    int Nf = (x.size() - Nxv) / 2;

    // add new feature positions to lookup dataAssociationTable
    for (unsigned int i = 0; i < idn.size(); i++) {
        dataAssociationTable[idn[i]] = Nf + i;
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
        ekfObserveModel(x, idf[i], zp, H_);
        H.block(i * 2, 0, 2, lenx) = H_;

        v(2 * i) = zf[i](0) - zp(0);
        v(2 * i + 1) = trigonometricOffset(zf[i](1) - zp(1));

        RR.block(i * 2, i * 2, 2, 2) = R;
    }

    choleskyUpdate(x, P, v, RR, H);
}

void EKFSLAM::ekfAddOneZ(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &Re) {
    int len;
    float r, b;
    float s, c;

    MatrixXf Gv, Gz;

    len = x.size();

    r = z(0);
    b = z(1);
    s = sin(x(2) + b);
    c = cos(x(2) + b);

    // Augment xEstimated xEstimated = [xEstimated; xEstimated(1)+r*c; xEstimated(2)+r*s];
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

    // Augment P
    P.conservativeResize(len + 2, len + 2);

    // Feature cov P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz';
    P.block(len, len, 2, 2) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() +
                              Gz * Re * Gz.transpose();

    // Vehicle to feature cross correlation
    //      P(rng,1:3)= Gv*P(1:3,1:3);
    //      P(1:3,rng)= P(rng,1:3)';
    P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
    P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();

    if (len > 3) {
        // Map to feature cross correlation P(rng,rnm)= Gv*P(1:3,rnm);
        P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);

        // P(rnm,rng)= P(rng,rnm)';
        P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
    }
}

void EKFSLAM::augment(VectorXf &x, MatrixXf &P, vector<VectorXf> &zn, MatrixXf &Re) {
    for (unsigned long i = 0; i < zn.size(); i++) {
        ekfAddOneZ(x, P, zn[i], Re);
    }
}