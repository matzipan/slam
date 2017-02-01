#ifndef __FASTSLAM_CORE_H__
#define __FASTSLAM_CORE_H__

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "utils.h"

#include "particle.h"

using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// SLAM config Variables
////////////////////////////////////////////////////////////////////////////////
class Conf : public CParamArray {
public:
    float V;
    float MAXG;
    float RATEG;
    float WHEELBASE;
    float DT_CONTROLS;

    float sigmaV;
    float sigmaG;

    float MAX_RANGE;
    float DT_OBSERVE;

    float sigmaR;
    float sigmaB;
    float sigmaT;


    float GATE_REJECT;
    float GATE_AUGMENT;

    float AT_WAYPOINT;
    int NUMBER_LOOPS;

    int NPARTICLES;
    int NEFFECTIVE;

    int SWITCH_CONTROL_NOISE;
    int SWITCH_SENSOR_NOISE;
    int SWITCH_INFLATE_NOISE;
    int SWITCH_PREDICT_NOISE;

    int SWITCH_SAMPLE_PROPOSAL;
    int SWITCH_HEADING_KNOWN;
    int SWITCH_RESAMPLE;
    int SWITCH_PROFILE;
    int SWITCH_SEED_RANDOM;

    int SWITCH_ASSOCIATION_KNOWN;
    int SWITCH_BATCH_UPDATE;
    int SWITCH_USE_IEKF;

public:
    // pase some data
    virtual int parse(void);
};

/**
 *
 * @param [in] x - true position
 * @param [in] waypoints
 * @param [in] indexOfFirstWaypoint - index to current waypoint
 * @param [in] minimumDistance - minimum distance to current waypoint before switching to next
 * @param [in/out] G - steering angle
 * @param [in] maximumSteeringAngleRate - maximum steering rate (radians/second)
 * @param [in] maximumSteeringAngle - maximum steering angle (radians)
 * @param [in] dt - timestep
 */
void updateSteering(VectorXf &x, MatrixXf &waypoints, int &indexOfFirstWaypoint, float minimumDistance, float &G,
                    float maximumSteeringAngleRate, float maximumSteeringAngle, float dt);

void predictTruePosition(VectorXf &x, float V, float G, float wheelBase, float dt);

vector<VectorXf> getObservations(MatrixXf landmarks, VectorXf &x, vector<int> &landmarkIdentifiers, float maximumRange);

void getVisibleLandmarks(MatrixXf &landmarks, VectorXf &x, vector<int> &landmarkIdentifiers, float maximumVisibilityRange);

vector<VectorXf> computeRangeBearing(MatrixXf &landmarks, VectorXf &x);

vector<int> findVisibleLandmarks(vector<float> &dx, vector<float> &dy, float vehicleAngle, float maximumVisibiltyRange);

void addControlNoise(float V, float G, MatrixXf &Q, float &Vn, float &Gn);

void addObservationNoise(vector<VectorXf> &z, MatrixXf &R);

void KF_joseph_update(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H);

void KF_cholesky_update(VectorXf &x, MatrixXf &P, VectorXf &v, MatrixXf &R, MatrixXf &H);


////////////////////////////////////////////////////////////////////////////////
// FastSLAM functions
////////////////////////////////////////////////////////////////////////////////
void compute_jacobians(Particle &particle, vector<int> &idf, MatrixXf &R, vector<VectorXf> &zp, vector<MatrixXf> *Hv, vector<MatrixXf> *Hf, vector<MatrixXf> *Sf);

void resample_particles(vector<Particle> &particles, int Nmin, bool do_resample);

void stratified_resample(VectorXf w, vector<int> &keep, float &Neff);

void cumulative_sum(VectorXf &w);

void data_associate_known(vector<VectorXf> &z, vector<int> &idz, VectorXf &data_association_table, int Nf, vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn);

void add_feature(Particle &particle, vector<VectorXf> &z, MatrixXf &R);

void feature_update(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);


/**
 * Reduce angle from a real number to offset on unit circle [0, 2*PI)
 * @param ang - angle in radians
 * @return angle in radians between [0, 2*PI)
 */
float trigonometricOffset(float ang);

MatrixXf make_symmetric(MatrixXf &P);

void transform_to_global(MatrixXf &p, VectorXf &b);

MatrixXf makeLaserLines(vector<VectorXf> &rb, VectorXf &x);

void makeCovarianceEllipse(MatrixXf &x, MatrixXf &P, MatrixXf &lines);

void stratified_random(unsigned long N, vector<float> &di);

double unifRand();

VectorXf multivariateGauss(VectorXf &x, MatrixXf &P, int n);

namespace nRandMat {
    MatrixXf randn(int m, int n);

    MatrixXf rand(int m, int n);
}

void readInputFile(const string s, MatrixXf *lm, MatrixXf *wp);


#endif // end of __FASTSLAM_CORE_H__

