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

class Conf : public CParamArray {
public:
    /// Speed in m/s
    float V;
    /// Maximum steering angle (-MAXG < g < MAXG) in radians
    float MAXG;
    /// Maximum rate of change for steer angle in radians/second
    float RATEG;
    /// Vehicle wheelbase in meters
    float WHEELBASE;
    /// Time interval between control signals in seconds
    float DT_CONTROLS;
    /// Speed control noise in meters/second
    float sigmaV;
    /// Steering control noise in radians
    float sigmaG;


    /// Maximum observation range
    float MAX_RANGE;
    /// Time interval between observations in seconds
    float DT_OBSERVE;
    /// Distance observation noise in meters
    float sigmaR;
    /// Angle observation noise in radians
    float sigmaB;
    /// Inertial measurement unit (IMU) angular noise in radians
    float sigmaT;

    // Data association/innovation gates (Mahalanobis distances)
    // For 2-D observation:
    //   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
    //   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.

    /// Maximum distance for data association.
    float GATE_REJECT;
    /// Minimum distance for creation of new feature
    float GATE_AUGMENT;

    /// Distance from current waypoint at which to switch to next waypoint
    float AT_WAYPOINT;
    /// Number of loops through the waypoint list
    int NUMBER_LOOPS;

    /// Number of particles
    int NPARTICLES;
    /// Minimum number of effective particles before resampling
    int NEFFECTIVE;


    int SWITCH_CONTROL_NOISE;
    int SWITCH_SENSOR_NOISE;
    int SWITCH_INFLATE_NOISE;
    /// Sample noise from predict (usually 1 for FastSLAM 1 and 0 for FastSLAM 2)
    int SWITCH_PREDICT_NOISE;

    /// Sample from proposal (no effect on FastSLAM 1 and usually 1 for FastSLAM 2)
    int SWITCH_SAMPLE_PROPOSAL;
    int SWITCH_HEADING_KNOWN;
    int SWITCH_RESAMPLE;
    int SWITCH_PROFILE;

    // If not 0, it is used as a seed for random number generation randn(), for repeatability
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

vector<int> findVisibleLandmarks(MatrixXf &landmarks, VectorXf &x, float maximumVisibiltyRange);

void addControlNoise(float V, float G, MatrixXf &Q, float &Vn, float &Gn);

void addObservationNoise(vector<VectorXf> &z, MatrixXf &R);

void josephUpdate(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H);

void choleskyUpdate(VectorXf &x, MatrixXf &P, VectorXf &v, MatrixXf &R, MatrixXf &H);


////////////////////////////////////////////////////////////////////////////////
// FastSLAM functions
////////////////////////////////////////////////////////////////////////////////
void computeJacobians(Particle &particle, vector<int> &idf, MatrixXf &R, vector<VectorXf> &zp, vector<MatrixXf> *Hv,
                      vector<MatrixXf> *Hf, vector<MatrixXf> *Sf);

void resampleParticles(vector<Particle> &particles, int nMin, bool doResample);

void stratifiedResample(VectorXf w, vector<int> &keep, float &Neff);

void cumulativeSum(VectorXf &w);

void dataAssociationKnown(vector<VectorXf> &z, vector<int> &idz, VectorXf &dataAssociationTable, int Nf,
                          vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn);

void addFeature(Particle &particle, vector<VectorXf> &z, MatrixXf &R);

void featureUpdate(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);


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

void stratifiedRandom(unsigned long N, vector<float> &di);

double unifRand();

VectorXf multivariateGauss(VectorXf &x, MatrixXf &P, int n);

namespace nRandMat {
    MatrixXf randn(int m, int n);

    MatrixXf rand(int m, int n);
}

void readInputFile(const string s, MatrixXf *lm, MatrixXf *wp);


#endif // end of __FASTSLAM_CORE_H__

