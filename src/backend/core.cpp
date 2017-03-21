#include <stdlib.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "core.h"

#ifdef JACOBIAN_ACCELERATOR
    #include "AcceleratorHandler.h"
#endif

using namespace std;
using namespace Eigen;

void addControlNoise(float V, float G, MatrixXf &Q, float &Vn, float &Gn) {
    VectorXf A(2);
    A(0) = V;
    A(1) = G;
    VectorXf C(2);
    C = multivariateGauss(A, Q, 1);
    Vn = C(0);
    Gn = C(1);
}


void predictTruePosition(VectorXf &x, float V, float G, float wheelBase, float dt) {
    x(0) = x(0) + V * dt * cos(G + x(2));
    x(1) = x(1) + V * dt * sin(G + x(2));
    x(2) = trigonometricOffset(x(2) + V * dt * sin(G) / wheelBase);
}

void updateSteering(VectorXf &x, MatrixXf &waypoints, int &indexOfFirstWaypoint, float minimumDistance, float &G,
                    float maximumSteeringAngleRate, float maximumSteeringAngle, float dt) {
    // Determine if current waypoint reached
    Vector2d currentWaypoint;
    currentWaypoint[0] = waypoints(0, indexOfFirstWaypoint);    //-1 since indexed from 0
    currentWaypoint[1] = waypoints(1, indexOfFirstWaypoint);

    float d2 = pow(currentWaypoint[0] - x[0], 2) + pow(currentWaypoint[1] - x[1], 2);

    if (d2 < minimumDistance * minimumDistance) {
        indexOfFirstWaypoint++; //switch to next
        if (indexOfFirstWaypoint >= waypoints.cols()) {
            indexOfFirstWaypoint = -1;
            return;
        }

        currentWaypoint[0] = waypoints(0, indexOfFirstWaypoint); // -1 since indexed from 0
        currentWaypoint[1] = waypoints(1, indexOfFirstWaypoint);
    }

    // Compute change in Gtrue to point towards current waypoint
    float deltaG = atan2(currentWaypoint[1] - x[1], currentWaypoint[0] - x[0]) - x[2] - G;
    deltaG = trigonometricOffset(deltaG);

    // Limit rate
    float maxDelta = maximumSteeringAngleRate * dt;
    if (abs(deltaG) > maxDelta) {
        int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
        deltaG = sign * maxDelta;
    }

    // Limit angle
    G = G + deltaG;
    if (abs(G) > maximumSteeringAngle) {
        int sign2 = (G > 0) ? 1 : ((G < 0) ? -1 : 0);
        G = sign2 * maximumSteeringAngle;
    }
}


/**
 * Find associations zf and new features zn
 * @param landmarksRangeBearing is range and bearing of visible landmarks
 * @param idz
 * @param dataAssociationTable
 * @param Nf
 * @param zf
 * @param idf
 * @param zn
 */
void dataAssociationKnown(vector<VectorXf> &landmarksRangeBearing, vector<int> &idz, VectorXf &dataAssociationTable, int Nf,
                          vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn) {
    vector<int> idn;

    zf.clear();
    zn.clear();
    idf.clear();

    for (unsigned long i = 0; i < idz.size(); i++) {
        unsigned long ii = idz[i];
        VectorXf z_i;

        if (dataAssociationTable(ii) == -1) {
            // New feature
            z_i = landmarksRangeBearing[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            // Existing feature
            z_i = landmarksRangeBearing[i];
            zf.push_back(z_i);
            idf.push_back(dataAssociationTable(ii));
        }
    }

    assert(idn.size() == zn.size());
    for (unsigned long i = 0; i < idn.size(); i++) {
        dataAssociationTable(idn[i]) = Nf + i;
    }
}


/**
 * Having selected a new pose from the proposal distribution, this pose is assumed perfect and each feature update maybe
 * computed independently and without pose uncertainty
 *
 * @param particle
 * @param z is the list of measurements conditioned on the particle.
 * @param idf
 * @param R
 */
void featureUpdate(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    // Updated EKF means
    vector<VectorXf> xf;
    // Updated EKF covariances
    vector<MatrixXf> Pf;

    for (unsigned long i = 0; i < idf.size(); i++) {
        // Means
        xf.push_back(particle.landmarkXs()[idf[i]]);
        // Covariances
        Pf.push_back(particle.landmarkPs()[idf[i]]);
    }

    vector<VectorXf> zp;
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    computeJacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    // Difference between two measurements (used to update mean)
    vector<VectorXf> v;
    for (unsigned long i = 0; i < z.size(); i++) {
        VectorXf measure_diff = z[i] - zp[i];
        measure_diff[1] = trigonometricOffset(measure_diff[1]);
        v.push_back(measure_diff);
    }

    VectorXf vi;
    MatrixXf Hfi;
    MatrixXf Pfi;
    VectorXf xfi;

    for (unsigned long i = 0; i < idf.size(); i++) {
        vi = v[i];
        Hfi = Hf[i];
        Pfi = Pf[i];
        xfi = xf[i];
        choleskyUpdate(xfi, Pfi, vi, R, Hfi);
        xf[i] = xfi;
        Pf[i] = Pfi;
    }

    for (unsigned long i = 0; i < idf.size(); i++) {
        particle.setLandmarkX(idf[i], xf[i]);
        particle.setLandmarkP(idf[i], Pf[i]);
    }
}

/**
 *
 * @param landmarks [in] - all landmarks
 * @param x [in] - position
 * @param landmarkIdentifiers - [in/out] inputs all landmark identifiers and outputs visible landmark identifiers
 * @param maximumRange - [in] maximum observation range
 * @return
 */
vector<VectorXf> getObservations(MatrixXf landmarks, VectorXf &x, vector<int> &landmarkIdentifiers, float maximumRange) {
    getVisibleLandmarks(landmarks, x, landmarkIdentifiers, maximumRange);
    return computeRangeBearing(landmarks, x);
}

/**
 *
 * @param landmarks [in] - all landmarks
 * @param x [in] - position
 * @param landmarkIdentifiers - [in/out] inputs all landmark identifiers and outputs visible landmark identifiers
 * @param maximumVisibilityRange - [in] maximum observation range
 */
void getVisibleLandmarks(MatrixXf &landmarks, VectorXf &x, vector<int> &landmarkIdentifiers, float maximumVisibilityRange) {
    // Eliminate distant points
    vector<int> visibleLandmarks = findVisibleLandmarks(landmarks, x, maximumVisibilityRange);

    MatrixXf landmarksNew(landmarks.rows(), visibleLandmarks.size());
    unsigned j, k;
    for (j = 0; j < landmarks.rows(); j++) {
        for (k = 0; k < visibleLandmarks.size(); k++) {
            landmarksNew(j, k) = landmarks(j, visibleLandmarks[k]);
        }
    }
    landmarks = MatrixXf(landmarksNew);

    vector<int> landmarkIdentifiersAux(landmarkIdentifiers);
    landmarkIdentifiers.clear();
    for (unsigned long i = 0; i < visibleLandmarks.size(); i++) {
        landmarkIdentifiers.push_back(landmarkIdentifiersAux[visibleLandmarks[i]]);
    }
}

vector<VectorXf> computeRangeBearing(MatrixXf &landmarks, VectorXf &x) {
    vector<float> dx;
    vector<float> dy;

    for (int c = 0; c < landmarks.cols(); c++) {
        dx.push_back(landmarks(0, c) - x(0));
        dy.push_back(landmarks(1, c) - x(1));
    }

    assert(dx.size() == landmarks.cols());
    assert(dy.size() == landmarks.cols());

    float vehicleAngle = x(2);
    vector<VectorXf> z;

    for (int i = 0; i < landmarks.cols(); i++) {
        VectorXf rangeBearing(2);
        rangeBearing << sqrt(pow(dx[i], 2) + pow(dy[i], 2)), atan2(dy[i], dx[i]) - vehicleAngle;
        z.push_back(rangeBearing);
    }

    return z;
}

/**
 * Incremental tests for bounding semi-circle.
 *
 * @param landmarks
 * @param x
 * @param vehicleAngle
 * @param maximumVisibiltyRange - [in] maximum observation range
 * @return
 */
vector<int> findVisibleLandmarks(MatrixXf &landmarks, VectorXf &x, float maximumVisibiltyRange) {
    // Select set of landmarks that are visible within vehicle's
    // Semi-circular field of view
    vector<float> dx;
    vector<float> dy;

    for (int c = 0; c < landmarks.cols(); c++) {
        dx.push_back(landmarks(0, c) - x(0));
        dy.push_back(landmarks(1, c) - x(1));
    }

    float vehicleAngle = x(2);

    vector<int> visibleLandmarks;

    for (unsigned long j = 0; j < dx.size(); j++) {
        if ((abs(dx[j]) < maximumVisibiltyRange) && (abs(dy[j]) < maximumVisibiltyRange)
            && ((dx[j] * cos(vehicleAngle) + dy[j] * sin(vehicleAngle)) > 0.0)
            && ((pow(dx[j], 2) + pow(dy[j], 2)) < pow(maximumVisibiltyRange, 2))) {
            visibleLandmarks.push_back(j);
        }
    }
    return visibleLandmarks;
}

void choleskyUpdate(VectorXf &x, MatrixXf &P, VectorXf &v, MatrixXf &R, MatrixXf &H) {
    MatrixXf PHt = P * H.transpose();
    MatrixXf S = H * PHt + R;

    // FIXME: why use conjugate()?
    // Make symmetric
    S = (S + S.transpose()) * 0.5;
    MatrixXf SChol = S.llt().matrixU();

    //  Tri matrix
    MatrixXf SCholInv = SChol.inverse();
    MatrixXf W1 = PHt * SCholInv;
    MatrixXf W = W1 * SCholInv.transpose();

    x = x + W * v;
    P = P - W1 * W1.transpose();
}


void josephUpdate(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H) {
    VectorXf PHt = P * H.transpose();
    MatrixXf S = H * PHt;
    MatrixXf _t = S;
    _t.setOnes();
    _t = _t * R;
    S = S + _t;
    MatrixXf Si = S.inverse();

    make_symmetric(Si);

    VectorXf W = PHt * Si;
    x = x + W * v;

    // Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W * H;
    P = C * P * C.transpose() + W * R * W.transpose();

    // Numerical safety
    float eps = 2.2204 * pow(10.0, -16);
    P = P + eye * eps;
}

MatrixXf make_symmetric(MatrixXf &P) {
    return (P + P.transpose()) * 0.5;
}


/**
 *
 * @param rb measurements
 * @param x robot pose
 * @return
 */
MatrixXf makeLaserLines(vector<VectorXf> &rb, VectorXf &x) {
    if (rb.empty()) {
        return MatrixXf(0, 0);
    }

    unsigned long len = rb.size();
    MatrixXf lines(4, len);

    MatrixXf globalMat(2, rb.size());
    int j;
    for (j = 0; j < globalMat.cols(); j++) {
        globalMat(0, j) = rb[j][0] * cos(rb[j][1]);
        globalMat(1, j) = rb[j][0] * sin(rb[j][1]);
    }

    transform_to_global(globalMat, x);

    for (int c = 0; c < lines.cols(); c++) {
        lines(0, c) = x(0);
        lines(1, c) = x(1);
        lines(2, c) = globalMat(0, c);
        lines(3, c) = globalMat(1, c);
    }

    return lines;
}


void makeCovarianceEllipse(MatrixXf &x, MatrixXf &P, MatrixXf &lines) {
    int i, N;
    float p_inc, p, cp, sp;
    float s;
    MatrixXf r;

    N = 16;
    p_inc = 2.0 * M_PI / N;
    s = 2.0;

    lines.setZero(2, N + 1);

    r = P.sqrt();

    for (i = 0; i <= N; i++) {
        p = i * p_inc;
        cp = cos(p);
        sp = sin(p);

        lines(0, i) = x(0) + s * (r(0, 0) * cp + r(0, 1) * sp);
        lines(1, i) = x(1) + s * (r(1, 0) * cp + r(1, 1) * sp);
    }
}

// http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
MatrixXf nRandMat::randn(int m, int n) {
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    int urows = m * n + 1;
    VectorXf u(urows);

    // u is a random matrix
    for (int r = 0; r < urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0 / RAND_MAX;
    }

    MatrixXf x(m, n);

    int i, j, k;
    float square, amp = 0, angle = 0;

    k = 0;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            if (k % 2 == 0) {
                square = -2. * std::log(u(k));
                if (square < 0.)
                    square = 0.;
                amp = sqrt(square);
                angle = 2. * M_PI * u(k + 1);
                x(i, j) = amp * std::sin(angle);
            } else {
                x(i, j) = amp * std::cos(angle);
            }

            k++;
        }
    }

    return x;
}

MatrixXf nRandMat::rand(int m, int n) {
    MatrixXf x(m, n);
    int i, j;
    float rand_max = float(RAND_MAX);

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++)
            x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
}

/**
 * Add random measurement noise.
 * @param z
 * @param R - we assume R is diagnoal matrix
 */
void addObservationNoise(vector<VectorXf> &z, MatrixXf &R) {
    unsigned long len = z.size();
    if (len > 0) {
        MatrixXf randM1 = nRandMat::randn(1, len);
        MatrixXf randM2 = nRandMat::randn(1, len);

        for (unsigned long c = 0; c < len; c++) {
            z[c][0] = z[c][0] + randM1(0, c) * sqrt(R(0, 0));
            z[c][1] = z[c][1] + randM2(0, c) * sqrt(R(1, 1));
        }
    }
}


VectorXf multivariateGauss(VectorXf &x, MatrixXf &P, int n) {
    // Cholesky decomposition
    MatrixXf S = P.llt().matrixL();
    MatrixXf X = nRandMat::randn(x.size(), n);

    return S * X + x;
}

float trigonometricOffset(float ang) {
    int n;

    if ((ang < -2 * M_PI) || (ang > 2 * M_PI)) {
        n = floor(ang / (2 * M_PI));
        ang = ang - n * (2 * M_PI);
    }

    if (ang > M_PI) {
        ang = ang - (2 * M_PI);
    }

    if (ang < -M_PI) {
        ang = ang + (2 * M_PI);
    }

    return ang;
}

void addFeature(Particle &particle, vector<VectorXf> &z, MatrixXf &R) {
    int lenZ = z.size();
    vector<VectorXf> xf;
    vector<MatrixXf> Pf;
    VectorXf xv = particle.xv();

    float r, b, s, c;
    MatrixXf Gz(2, 2);

    for (int i = 0; i < lenZ; i++) {
        r = z[i][0];
        b = z[i][1];
        s = sin(xv(2) + b);
        c = cos(xv(2) + b);

        VectorXf measurement(2);
        measurement(0) = xv(0) + r * c;
        measurement(1) = xv(1) + r * s;
        xf.push_back(measurement);
        Gz << c, -r * s, s, r * c;

        Pf.push_back(Gz * R * Gz.transpose());
    }

    int lenx = particle.landmarkXs().size();

    for (int i = lenx; i < lenx + lenZ; i++) {
        particle.setLandmarkX(i, xf[(i - lenx)]);
        particle.setLandmarkP(i, Pf[(i - lenx)]);
    }
}

/**
 *
 * @param particle
 * @param idf
 * @param R
 * @param zp - measurement (range, bearing)
 * @param Hv - Jacobians of function h (derivative of h wrt pose)
 * @param Hf - Jacobians of function h (derivative of h wrt mean)
 * @param Sf - measurement covariance
 */

#ifdef JACOBIAN_ACCELERATOR
extern AcceleratorHandler* acceleratorHandler;
#endif

void computeJacobians(Particle &particle, vector<int> &idf, MatrixXf &R, vector<VectorXf> &zp, vector<MatrixXf> *Hv,
                      vector<MatrixXf> *Hf, vector<MatrixXf> *Sf) {
#ifdef JACOBIAN_ACCELERATOR
    VectorXf xv = particle.xv();

    int current_memory_write_position = 0;
    int current_memory_read_position = 0;
    uint32_t n = idf.size();

    float* acceleratorData = (float*) acceleratorHandler->getMemoryPointer();


     for (int i = 0; i < 3; i++) {
         acceleratorData[current_memory_write_position++] = xv(i);
     }

    for (int i = 0; i < 4; i++) {
        acceleratorData[current_memory_write_position++] = R(i);	// R[i][j]
    }

    vector<VectorXf> landmarkXs = particle.landmarkXs();
    vector<MatrixXf> landmarkPs = particle.landmarkPs();


    for (int i = 0; i < n; i++) {
    	for (int j = 0; j < 2; j++) {
            acceleratorData[current_memory_write_position++] = landmarkXs[idf[i]](j);	// xf[i]
    	}

    	for (int j = 0; j < 4; j++) {
            acceleratorData[current_memory_write_position++] = landmarkPs[idf[i]](j);	// Pf[i][j]
		}

    }

    acceleratorHandler->setN(n);
    acceleratorHandler->start();

    while (!acceleratorHandler->isDone());

    current_memory_read_position = 3+4+(2+4)*n;

    MatrixXf HvMat(2, 3);
    MatrixXf HfMat(2, 2);
    MatrixXf SfMat(2, 2);
    VectorXf zpVec(2);

    for (int i = 0; i < n; i++) {
        HfMat << acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++];

        // Jacobian wrt. vehicle states
        HvMat << acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++];

        SfMat << acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++],
                acceleratorData[current_memory_read_position++];

        zp.push_back(zpVec);
        Hv->push_back(HvMat);
        Hf->push_back(HfMat);
        Sf->push_back(SfMat);
    }
#else
    VectorXf xv = particle.xv();

#ifdef DATA_DUMP
    printf("-----------\nxv %.10f %.10f %.10f\n", xv[0], xv[1], xv[2]);
    printf("R %.10f %.10f %.10f %.10f\n", R(0,0),R(0,1), R(1,0), R(1,1));
    printf("idf.size %u\n", idf.size());
#endif

    vector<VectorXf> xf;
    // Particle Pf is a array of matrices
    vector<MatrixXf> Pf;

    for (unsigned int i = 0; i < idf.size(); i++) {
#ifdef DATA_DUMP
        printf("xf %.10f %.10f\n", particle.landmarkXs()[idf[i]][0], particle.landmarkXs()[idf[i]][1]);
        printf("Pf %.10f %.10f %.10f %.10f\n", particle.landmarkPs()[idf[i]](0, 0), particle.landmarkPs()[idf[i]](0, 1), particle.landmarkPs()[idf[i]](1, 0), particle.landmarkPs()[idf[i]](1,1));
#endif

        xf.push_back(particle.landmarkXs()[idf[i]]);
        Pf.push_back((particle.landmarkPs())[idf[i]]);
    }

    float dx, dy, d2, d;
    MatrixXf HvMat(2, 3);
    MatrixXf HfMat(2, 2);
    VectorXf zpVec(2);

    for (unsigned int i = 0; i < idf.size(); i++) {
        dx = xf[i](0) - xv(0); // maximum for d is observation range
        dy = xf[i](1) - xv(1); // maximum for d is observation range
        d2 = pow(dx, 2) + pow(dy, 2); // I believe maximum for d2 is (observation range)^2
        d = sqrt(d2); // maximum for d is observation range

        // Predicted observation
        zpVec[0] = d;
        zpVec[1] = trigonometricOffset(atan2(dy, dx) - xv(2)); // [0, 6.28) range
        zp.push_back(zpVec);

        // Jacobian wrt vehicle states
        HvMat << -dx / d, -dy / d, 0, dy / d2, -dx / d2, -1;

        // Jacobian wrt feature states
        HfMat << dx / d, dy / d, -dy / d2, dx / d2;

        Hv->push_back(HvMat);
        Hf->push_back(HfMat);

        // Innovation covariance of feature observation given the vehicle'
        MatrixXf SfMat = HfMat * Pf[i] * HfMat.transpose() + R;
        Sf->push_back(SfMat);

#ifdef DATA_DUMP
        printf("dx %.10f dy %.10f d2 %.10f d %.10f\n", dx, dy, d2, d);
        printf("zp %.10f %.10f\n", zpVec[0], zpVec[1]);
        printf("Hf %.10f %.10f %.10f %.10f\n", HfMat(0,0), HfMat(0,1), HfMat(1,0), HfMat(1,1));
        printf("Hv %.10f %.10f %.10f %.10f %.10f %.10f\n", HvMat(0,0), HvMat(0,1), HvMat(0,2), HvMat(1,0), HvMat(1,1), HvMat(1,2));
        printf("Sf %.10f %.10f %.10f %.10f\n", SfMat(0,0), SfMat(0,1), SfMat(1,0), SfMat(1,1));
#endif
    }
#endif
}


void resampleParticles(vector<Particle> &particles, int nMin, bool doResample) {
    unsigned long i;
    VectorXf w(particles.size());

    for (i = 0; i < particles.size(); i++) {
        w(i) = particles[i].w();
    }

    float ws = w.sum();
    for (i = 0; i < particles.size(); i++) {
        particles[i].setW(w(i) / ws);
    }

    float nEff = 0;
    vector<int> keep;

    stratifiedResample(w, keep, nEff);

    vector<Particle> oldParticles = vector<Particle>(particles);
    particles.resize(keep.size());

    if (doResample && (nEff < nMin)) {
        for (i = 0; i < keep.size(); i++) {
            particles[i] = oldParticles[keep[i]];
        }

        for (i = 0; i < particles.size(); i++) {
            float newW = 1.0f / (float) particles.size();
            particles[i].setW(newW);
        }
    }
}

void stratifiedRandom(unsigned long N, vector<float> &di) {
    float k = 1.0 / (float) N;

    // Deterministic intervals
    float temp = k / 2;
    while (temp < (1 - k / 2)) {
        di.push_back(temp);
        temp = temp + k;
    }

    // FIXME: when set NPARTICLES = 30, this will fail
    assert(di.size() == N);

    // Dither within interval
    vector<float>::iterator diter;
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k / 2);
    }
}

/**
 * Generate a random number between 0 and 1
 * @return random number in [0, 1]
 */
double unifRand() {
    return rand() / double(RAND_MAX);
}

// FIXME: input w will be modified?
void stratifiedResample(VectorXf w, vector<int> &keep, float &nEff) {
    VectorXf wSqrd(w.size());
    float wSum = w.sum();

    for (int i = 0; i < w.size(); i++) {
        w(i) = w(i) / wSum;
        wSqrd(i) = pow(w(i), 2);
    }
    nEff = 1 / wSqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i = 0; i < len; i++) {
        keep[i] = -1;
    }

    vector<float> select;
    stratifiedRandom(len, select);
    cumulativeSum(w);

    int ctr = 0;
    for (int i = 0; i < len; i++) {
        while ((ctr < len) && (select[ctr] < w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}

/**
 *
 * @param w [in/out] a cumulative sum array
 */
void cumulativeSum(VectorXf &w) {
    VectorXf csumVec(w.size());
    for (int i = 0; i < w.size(); i++) {
        float sum = 0;
        for (int j = 0; j <= i; j++) {
            sum += w(j);
        }
        csumVec(i) = sum;
    }

    w = VectorXf(csumVec); // Copy constructor. Double check
}


void transform_to_global(MatrixXf &p, VectorXf &b) {
    // rotate
    MatrixXf rot(2, 2);
    rot << cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));

    MatrixXf p_resized;
    p_resized = MatrixXf(p);
    p_resized.conservativeResize(2, p_resized.cols());
    p_resized = rot * p_resized;

    // translate
    int c;
    for (c = 0; c < p_resized.cols(); c++) {
        p(0, c) = p_resized(0, c) + b(0);
        p(1, c) = p_resized(1, c) + b(1);
    }

    float input;
    // if p is a pose and not a point
    if (p.rows() == 3) {
        for (int k = 0; k < p_resized.cols(); k++) {
            input = p(2, k) + b(2);
            p(2, k) = trigonometricOffset(input);
        }
    }
}


void readInputFile(const string s, MatrixXf *lm, MatrixXf *wp) {
    if (access(s.c_str(), R_OK) == -1) {
        std::cerr << "Unable to read input file" << s << std::endl;
        exit(EXIT_FAILURE);
    }

    ifstream in(s.c_str());

    int lineno = 0;
    unsigned long lm_rows = 0;
    int lm_cols = 0;
    unsigned long wp_rows = 0;
    unsigned long wp_cols = 0;

    while (in) {
        lineno++;
        string str;
        getline(in, str);
        istringstream line(str);

        vector<string> tokens;
        copy(istream_iterator<string>(line),
             istream_iterator<string>(),
             back_inserter<vector<string> >(tokens));

        if (tokens.size() == 0) {
            continue;
        }
        else if (tokens[0][0] == '#') {
            continue;
        }
        else if (tokens[0] == "lm") {
            if (tokens.size() != 3) {
                std::cerr << "Wrong args for lm!" << std::endl;
                std::cerr << "Error occuredon line" << lineno << std::endl;
                std::cerr << "line:" << str << std::endl;
                exit(EXIT_FAILURE);
            }
            lm_rows = strtof(tokens[1].c_str(), NULL);
            lm_cols = strtof(tokens[2].c_str(), NULL);

            lm->resize(lm_rows, lm_cols);
            for (int c = 0; c < lm_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr << "EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in, str);
                istringstream line(str);
                vector<string> tokens;
                copy(istream_iterator<string>(line),
                     istream_iterator<string>(),
                     back_inserter<vector<string> >(tokens));
                if (tokens.size() < lm_rows) {
                    std::cerr << "invalid line for lm coordinate!" << std::endl;
                    std::cerr << "Error occured on line " << lineno << std::endl;
                    std::cerr << "line: " << str << std::endl;
                    exit(EXIT_FAILURE);
                }

                for (unsigned long r = 0; r < lm_rows; r++) {
                    (*lm)(r, c) = strtof(tokens[r].c_str(), NULL);
                }
            }
        }
        else if (tokens[0] == "wp") {
            if (tokens.size() != 3) {
                std::cerr << "Wrong args for wp!" << std::endl;
                std::cerr << "Error occured on line" << lineno << std::endl;
                std::cerr << "line:" << str << std::endl;
                exit(EXIT_FAILURE);
            }
            wp_rows = strtof(tokens[1].c_str(), NULL);
            wp_cols = strtof(tokens[2].c_str(), NULL);
            wp->resize(wp_rows, wp_cols);
            for (unsigned long c = 0; c < wp_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr << "EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in, str);
                istringstream waypoint_line(str);
                std::vector<string> waypoint_tokens;
                copy(istream_iterator<string>(waypoint_line),
                     istream_iterator<string>(),
                     back_inserter<std::vector<string> >(waypoint_tokens));
                if (waypoint_tokens.size() < wp_rows) {
                    std::cerr << "invalid waypoint_line for wp coordinate!" << std::endl;
                    std::cerr << "Error occured on waypoint_line " << lineno << std::endl;
                    std::cerr << "waypoint_line: " << str << std::endl;
                    exit(EXIT_FAILURE);
                }

                for (unsigned long r = 0; r < lm_rows; r++) {
                    (*wp)(r, c) = strtof(waypoint_tokens[r].c_str(), NULL);
                }
            }
        }
        else {
            std::cerr << "Unkwown command" << tokens[0] << std::endl;
            std::cerr << "Error occured on line" << lineno << std::endl;
            std::cerr << "line: " << str << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// fastslam config Variables
//      Configuration file
//      Permits various adjustments to parameters of the FastSLAM algorithm.
//      See fastslam_sim.h for more information
////////////////////////////////////////////////////////////////////////////////

int Conf::parse(void) {
    ////////////////////////////////////////////////////////////////////////////
    /// set default values
    ////////////////////////////////////////////////////////////////////////////

    // control parameters
    V = 3.0;              // m/s
    MAXG = 30 * M_PI / 180;      // radians, maximum steering angle (-MAXG < g < MAXG)
    RATEG = 20 * M_PI / 180;      // rad/s, maximum rate of change in steer angle
    WHEELBASE = 4;                // metres, vehicle wheel-base
    DT_CONTROLS = 0.025;            // seconds, time interval between control signals

    // control noises
    sigmaV = 0.3;              // m/s
    sigmaG = (3.0 * M_PI / 180);   // radians


    // observation parameters
    MAX_RANGE = 30.0;                     // metres
    DT_OBSERVE = 8 * DT_CONTROLS;            // seconds, time interval between observations

    // observation noises
    sigmaR = 0.1;                      // metres
    sigmaB = (1.0 * M_PI / 180);           // radians
    sigmaT = (1.0 * M_PI / 180);           // IMU angular noise (radians)


    // data association innovation gates (Mahalanobis distances)
    GATE_REJECT = 4.0;                      // maximum distance for association
    GATE_AUGMENT = 25.0;                     // minimum distance for creation of new feature
    // For 2-D observation:
    //   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
    //   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.


    // waypoint proximity
    AT_WAYPOINT = 1.0;                      // metres, distance from current waypoint at which to switch to next waypoint
    NUMBER_LOOPS = 2;                        // number of loops through the waypoint list

    // resampling
    NPARTICLES = 100;
    NEFFECTIVE = 0.75 * NPARTICLES;          // minimum number of effective particles before resampling

    // switches
    SWITCH_CONTROL_NOISE = 1;
    SWITCH_SENSOR_NOISE = 1;
    SWITCH_INFLATE_NOISE = 0;
    SWITCH_PREDICT_NOISE = 0;    // sample noise from predict (usually 1 for fastslam1.0 and 0 for fastslam2.0)
    SWITCH_SAMPLE_PROPOSAL = 1;    // sample from proposal (no effect on fastslam1.0 and usually 1 for fastslam2.0)
    SWITCH_HEADING_KNOWN = 1;
    SWITCH_RESAMPLE = 1;
    SWITCH_PROFILE = 1;
    SWITCH_SEED_RANDOM = 0;    // if not 0, seed the randn() with its value at beginning //
    //  of simulation (for repeatability)

    SWITCH_ASSOCIATION_KNOWN = 0;
    SWITCH_BATCH_UPDATE = 1;
    SWITCH_USE_IEKF = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// parse values
    ////////////////////////////////////////////////////////////////////////////
    f("Vtrue", V);
    f("MAXG", MAXG);
    f("RATEG", RATEG);
    f("WHEELBASE", WHEELBASE);
    f("DT_CONTROLS", DT_CONTROLS);

    f("sigmaV", sigmaV);
    f("sigmaG", sigmaG);

    f("MAX_RANGE", MAX_RANGE);
    f("DT_OBSERVE", DT_OBSERVE);

    f("sigmaR", sigmaR);
    f("sigmaB", sigmaB);
    f("sigmaT", sigmaT);

    f("GATE_REJECT", GATE_REJECT);
    f("GATE_AUGMENT", GATE_AUGMENT);

    f("AT_WAYPOINT", AT_WAYPOINT);
    i("NUMBER_LOOPS", NUMBER_LOOPS);

    i("NPARTICLES", NPARTICLES);
    i("NEFFECTIVE", NEFFECTIVE);

    i("SWITCH_CONTROL_NOISE", SWITCH_CONTROL_NOISE);
    i("SWITCH_SENSOR_NOISE", SWITCH_SENSOR_NOISE);
    i("SWITCH_INFLATE_NOISE", SWITCH_INFLATE_NOISE);
    i("SWITCH_PREDICT_NOISE", SWITCH_PREDICT_NOISE);
    i("SWITCH_SAMPLE_PROPOSAL", SWITCH_SAMPLE_PROPOSAL);
    i("SWITCH_HEADING_KNOWN", SWITCH_HEADING_KNOWN);
    i("SWITCH_RESAMPLE", SWITCH_RESAMPLE);
    i("SWITCH_PROFILE", SWITCH_PROFILE);
    i("SWITCH_SEED_RANDOM", SWITCH_SEED_RANDOM);

    i("SWITCH_ASSOCIATION_KNOWN", SWITCH_ASSOCIATION_KNOWN);
    i("SWITCH_BATCH_UPDATE", SWITCH_BATCH_UPDATE);
    i("SWITCH_USE_IEKF", SWITCH_USE_IEKF);

    return 0;
}
