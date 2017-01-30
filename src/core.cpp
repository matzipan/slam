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
#include "particle.h"

using namespace std;
using namespace Eigen;

void add_control_noise(float V, float G, MatrixXf &Q, int add_noise, float *VnGn) {
    if (add_noise == 1) {
        VectorXf A(2);
        A(0) = V;
        A(1) = G;
        VectorXf C(2);
        C = multivariate_gauss(A, Q, 1);
        VnGn[0] = C(0);
        VnGn[1] = C(1);
    }
}


void predict_true(VectorXf &xv, float V, float G, float WB, float dt) {
    xv(0) = xv(0) + V * dt * cos(G + xv(2));
    xv(1) = xv(1) + V * dt * sin(G + xv(2));
    xv(2) = pi_to_pi(xv(2) + V * dt * sin(G) / WB);
}
/**
 *
 * @param[in] xTrue - true position
 * @param[in] waypoints
 * @param[in] indexOfFirstWaypoint - index to current waypoint
 * @param[in] minimumDistance - minimum distance to current waypoint before switching to next
 * @param[out] G - output - current steering angle
 * @param[in] rateG - maximum steering rate (radians/second)
 * @param[in] maxG - maximum steering angle (radians)
 * @param[in] dt - timestep
 */
void compute_steering(VectorXf &xTrue, MatrixXf &waypoints, int &indexOfFirstWaypoint, float minimumDistance, float &G, float rateG, float maxG, float dt) {
    // Determine if current waypoint reached
    Vector2d currentWaypoint;
    currentWaypoint[0] = waypoints(0, indexOfFirstWaypoint);    //-1 since indexed from 0
    currentWaypoint[1] = waypoints(1, indexOfFirstWaypoint);

    float d2 = pow(currentWaypoint[0] - xTrue[0], 2) + pow(currentWaypoint[1] - xTrue[1], 2);

    if (d2 < minimumDistance * minimumDistance) {
        indexOfFirstWaypoint++; //switch to next
        if (indexOfFirstWaypoint >= waypoints.cols()) {
            indexOfFirstWaypoint = -1;
            return;
        }

        currentWaypoint[0] = waypoints(0, indexOfFirstWaypoint); // -1 since indexed from 0
        currentWaypoint[1] = waypoints(1, indexOfFirstWaypoint);
    }

    // Compute change in G to point towards current waypoint
    float deltaG = atan2(currentWaypoint[1] - xTrue[1], currentWaypoint[0] - xTrue[0]) - xTrue[2] - G;
    deltaG = pi_to_pi(deltaG);

    // Limit rate
    float maxDelta = rateG * dt;
    if (abs(deltaG) > maxDelta) {
        int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
        deltaG = sign * maxDelta;
    }

    // Limit angle
    G = G + deltaG;
    if (abs(G) > maxG) {
        int sign2 = (G > 0) ? 1 : ((G < 0) ? -1 : 0);
        G = sign2 * maxG;
    }
}


//z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void data_associate_known(vector<VectorXf> &z, vector<int> &idz, VectorXf &data_association_table, int Nf, vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn) {
    idf.clear();
    vector<int> idn;

    unsigned long ii;

    for (unsigned long i = 0; i < idz.size(); i++) {
        ii = idz[i];
        VectorXf z_i;
        if (data_association_table(ii) == -1) { //new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        }
        else {
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(data_association_table(ii));
        }
    }

    assert(idn.size() == zn.size());
    for (unsigned long i = 0; i < idn.size(); i++) {
        data_association_table(idn[i]) = Nf + i;
    }
}


//z is the list of measurements conditioned on the particle.
void feature_update(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R) {
    //Having selected a new pose from the proposal distribution,
    //  this pose is assumed perfect and each feature update maybe
    //  computed independently and without pose uncertainty
    vector<VectorXf> xf;    //updated EKF means
    vector<MatrixXf> Pf;    //updated EKF covariances

    for (unsigned long i = 0; i < idf.size(); i++) {
        xf.push_back(particle.xf()[idf[i]]); //means
        Pf.push_back(particle.Pf()[idf[i]]); //covariances
    }

    vector<VectorXf> zp;
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    compute_jacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    vector<VectorXf> v; //difference btw two measurements (used to update mean)
    for (unsigned long i = 0; i < z.size(); i++) {
        VectorXf measure_diff = z[i] - zp[i];
        measure_diff[1] = pi_to_pi(measure_diff[1]);
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
        KF_cholesky_update(xfi, Pfi, vi, R, Hfi);
        xf[i] = xfi;
        Pf[i] = Pfi;
    }

    for (unsigned long i = 0; i < idf.size(); i++) {
        particle.setXfi(idf[i], xf[i]);
        particle.setPfi(idf[i], Pf[i]);
    }
}

vector<VectorXf> get_observations(VectorXf &x, MatrixXf lm, vector<int> &idf, float rmax) {
    get_visible_landmarks(x, lm, idf, rmax);
    return compute_range_bearing(x, lm);
}

void get_visible_landmarks(VectorXf &x, MatrixXf &lm, vector<int> &idf, float rmax) {
    //select set of landmarks that are visible within vehicle's
    //semi-circular field of view
    vector<float> dx;
    vector<float> dy;

    for (int c = 0; c < lm.cols(); c++) {
        dx.push_back(lm(0, c) - x(0));
        dy.push_back(lm(1, c) - x(1));
    }

    float phi = x(2);

    //distant points are eliminated
    vector<int> ii = find2(dx, dy, phi, rmax);

    MatrixXf lm_new(lm.rows(), ii.size());
    unsigned j, k;
    for (j = 0; j < lm.rows(); j++) {
        for (k = 0; k < ii.size(); k++) {
            lm_new(j, k) = lm(j, ii[k]);
        }
    }
    lm = MatrixXf(lm_new);

    vector<int> idf_backup(idf);
    idf.clear();
    for (unsigned long i = 0; i < ii.size(); i++) {
        idf.push_back(idf_backup[ii[i]]);
    }
}

vector<VectorXf> compute_range_bearing(VectorXf &x, MatrixXf &lm) {
    vector<float> dx;
    vector<float> dy;

    for (int c = 0; c < lm.cols(); c++) {
        dx.push_back(lm(0, c) - x(0));
        dy.push_back(lm(1, c) - x(1));
    }

    assert(dx.size() == lm.cols());
    assert(dy.size() == lm.cols());

    float phi = x(2);
    vector<VectorXf> z;

    for (int i = 0; i < lm.cols(); i++) {
        VectorXf zvec(2);
        zvec << sqrt(pow(dx[i], 2) + pow(dy[i], 2)), atan2(dy[i], dx[i]) - phi;
        z.push_back(zvec);
    }

    return z;
}

vector<int> find2(vector<float> &dx, vector<float> &dy, float phi, float rmax) {
    vector<int> index;
    //incremental tests for bounding semi-circle
    for (unsigned long j = 0; j < dx.size(); j++) {
        if ((abs(dx[j]) < rmax) && (abs(dy[j]) < rmax)
            && ((dx[j] * cos(phi) + dy[j] * sin(phi)) > 0.0)
            && ((pow(dx[j], 2) + pow(dy[j], 2)) < pow(rmax, 2))) {
            index.push_back(j);
        }
    }
    return index;
}

void KF_cholesky_update(VectorXf &x, MatrixXf &P, VectorXf &v, MatrixXf &R, MatrixXf &H) {
    MatrixXf PHt = P * H.transpose();
    MatrixXf S = H * PHt + R;

    // FIXME: why use conjugate()?
    S = (S + S.transpose()) * 0.5; // Make symmetric
    MatrixXf SChol = S.llt().matrixU();

    MatrixXf SCholInv = SChol.inverse(); //tri matrix
    MatrixXf W1 = PHt * SCholInv;
    MatrixXf W = W1 * SCholInv.transpose();

    x = x + W * v;
    P = P - W1 * W1.transpose();
}


void KF_joseph_update(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H) {
    VectorXf PHt = P * H.transpose();
    MatrixXf S = H * PHt;
    MatrixXf _t = S;
    _t.setOnes();
    _t = _t * R;
    S = S + _t;
    MatrixXf Si = S.inverse();
    //Si = make_symmetric(Si);
    make_symmetric(Si);
    MatrixXf PSD_check = Si.llt().matrixU(); //chol of scalar is sqrt
    //PSD_check.transpose();
    //PSD_check.conjugate();

    VectorXf W = PHt * Si;
    x = x + W * v;

    //Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W * H;
    P = C * P * C.transpose() + W * R * W.transpose();

    float eps = 2.2204 * pow(10.0, -16); //numerical safety
    P = P + eye * eps;

    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXf make_symmetric(MatrixXf &P) {
    return (P + P.transpose()) * 0.5;
}


MatrixXf line_plot_conversion(MatrixXf &lnes) {
    int len = lnes.cols() * 3 - 1;
    MatrixXf p(2, len);

    for (int j = 0; j < len; j += 3) {
        int k = floor((j + 1) / 3); //reverse of len calculation
        p(0, j) = lnes(0, k);
        p(1, j) = lnes(1, k);
        p(0, j + 1) = lnes(2, k);
        p(1, j + 1) = lnes(3, k);
        if (j + 2 < len) {
            p(0, j + 2) = 0;
            p(1, j + 2) = 0;
        }
    }
    return p;
}

// rb is measurements
// xv is robot pose
MatrixXf make_laser_lines(vector<VectorXf> &rb, VectorXf &xv) {
    if (rb.empty()) {
        return MatrixXf(0, 0);
    }

    unsigned long len = rb.size();
    MatrixXf lnes(4, len);

    MatrixXf globalMat(2, rb.size());
    int j;
    for (j = 0; j < globalMat.cols(); j++) {
        globalMat(0, j) = rb[j][0] * cos(rb[j][1]);
        globalMat(1, j) = rb[j][0] * sin(rb[j][1]);
    }

    transform_to_global(globalMat, xv);

    for (int c = 0; c < lnes.cols(); c++) {
        lnes(0, c) = xv(0);
        lnes(1, c) = xv(1);
        lnes(2, c) = globalMat(0, c);
        lnes(3, c) = globalMat(1, c);
    }

    //return line_plot_conversion(lnes);
    return lnes;
}


void make_covariance_ellipse(MatrixXf &x, MatrixXf &P, MatrixXf &lines) {
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

//add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(vector<VectorXf> &z, MatrixXf &R, int addnoise) {
    if (addnoise == 1) {
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
}


VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n) {
    int len = x.size();

    //choleksy decomposition
    MatrixXf S = P.llt().matrixL();
    MatrixXf X = nRandMat::randn(len, n);

    return S * X + x;
}

float pi_to_pi(float ang) {
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

float pi_to_pi2(float ang) {
    if (ang > M_PI) {
        ang = ang - (2 * M_PI);
    }

    if (ang < -M_PI) {
        ang = ang + (2 * M_PI);
    }

    return ang;
}


//
// add new features
//
void add_feature(Particle &particle, vector<VectorXf> &z, MatrixXf &R) {
    int lenz = z.size();
    vector<VectorXf> xf;
    vector<MatrixXf> Pf;
    VectorXf xv = particle.xv();

    float r, b, s, c;
    MatrixXf Gz(2, 2);

    for (int i = 0; i < lenz; i++) {
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

    int lenx = particle.xf().size();

    for (int i = lenx; i < lenx + lenz; i++) {
        particle.setXfi(i, xf[(i - lenx)]);
        particle.setPfi(i, Pf[(i - lenx)]);
    }
}

void compute_jacobians(Particle &particle, vector<int> &idf, MatrixXf &R, vector<VectorXf> &zp, vector<MatrixXf> *Hv, vector<MatrixXf> *Hf, vector<MatrixXf> *Sf) {
    // zp - measurement (range, bearing)
    // Hv - jacobians of function h (deriv of h wrt pose)
    // Hf - jacobians of function h (deriv of h wrt mean)
    // Sf - measurement covariance

    VectorXf xv = particle.xv();

    vector<VectorXf> xf;
    vector<MatrixXf> Pf;

    unsigned int i;
    for (i = 0; i < idf.size(); i++) {
        xf.push_back(particle.xf()[idf[i]]);
        Pf.push_back((particle.Pf())[idf[i]]); //particle.Pf is a array of matrices
    }

    float dx, dy, d2, d;
    MatrixXf HvMat(2, 3);
    MatrixXf HfMat(2, 2);

    for (i = 0; i < idf.size(); i++) {
        dx = xf[i](0) - xv(0);
        dy = xf[i](1) - xv(1);
        d2 = pow(dx, 2) + pow(dy, 2);
        d = sqrt(d2);

        VectorXf zp_vec(2);

        //predicted observation
        zp_vec[0] = d;
        zp_vec[1] = pi_to_pi(atan2(dy, dx) - xv(2));
        zp.push_back(zp_vec);

        //Jacobian wrt vehicle states
        HvMat << -dx / d, -dy / d, 0, dy / d2, -dx / d2, -1;

        //Jacobian wrt feature states
        HfMat << dx / d, dy / d, -dy / d2, dx / d2;

        Hv->push_back(HvMat);
        Hf->push_back(HfMat);

        //innovation covariance of feature observation given the vehicle'
        MatrixXf SfMat = HfMat * Pf[i] * HfMat.transpose() + R;
        Sf->push_back(SfMat);
    }
}


void resample_particles(vector<Particle> &particles, int Nmin, bool do_resample) {
    unsigned long i;
    VectorXf w(particles.size());

    for (i = 0; i < particles.size(); i++) {
        w(i) = particles[i].w();
    }

    float ws = w.sum();
    for (i = 0; i < particles.size(); i++) {
        particles[i].setW(w(i) / ws);
    }

    float Neff = 0;
    vector<int> keep;

    stratified_resample(w, keep, Neff);

    vector<Particle> old_particles = vector<Particle>(particles);
    particles.resize(keep.size());

    if (do_resample && (Neff < Nmin)) {
        for (i = 0; i < keep.size(); i++) {
            particles[i] = old_particles[keep[i]];
        }

        for (i = 0; i < particles.size(); i++) {
            float new_w = 1.0f / (float) particles.size();
            particles[i].setW(new_w);
        }
    }
}

void stratified_random(unsigned long N, vector<float> &di) {
    float k = 1.0 / (float) N;

    //deterministic intervals
    float temp = k / 2;
    while (temp < (1 - k / 2)) {
        di.push_back(temp);
        temp = temp + k;
    }

    // FIXME: when set NPARTICLES = 30, this whill failed
    assert(di.size() == N);

    //dither within interval
    vector<float>::iterator diter;
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k / 2);
    }
}

//
// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand() {
    return rand() / double(RAND_MAX);
}

// FIXME: input w will be modified?
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff) {
    VectorXf wsqrd(w.size());
    double w_sum = w.sum();

    for (int i = 0; i < w.size(); i++) {
        // FIXME: matlab version is: w = w / sum(w)
        w(i) = w(i) / w_sum;
        wsqrd(i) = pow(w(i), 2);
    }
    Neff = 1 / wsqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i = 0; i < len; i++) {
        keep[i] = -1;
    }

    vector<float> select;
    stratified_random(len, select);
    cumulative_sum(w);

    int ctr = 0;
    for (int i = 0; i < len; i++) {
        while ((ctr < len) && (select[ctr] < w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}

// returns a cumulative sum array
void cumulative_sum(VectorXf &w) {
    VectorXf csumVec(w.size());
    for (int i = 0; i < w.size(); i++) {
        float sum = 0;
        for (int j = 0; j <= i; j++) {
            sum += w(j);
        }
        csumVec(i) = sum;
    }

    w = VectorXf(csumVec); // copy constructor. Double check
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
            p(2, k) = pi_to_pi(input);
        }
    }
}


void read_slam_input_file(const string s, MatrixXf *lm, MatrixXf *wp) {
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
    AT_WAYPOINT = 1.0;                      // metres, distance from current
    // waypoint at which to switch to next waypoint
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
    f("V", V);
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
