//
// Created by matzipan on 11/11/16.
//

#ifndef SLAM_GUI_PARTICLE_H
#define SLAM_GUI_PARTICLE_H

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class Particle {
public:
    Particle();

    Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &xf, vector<MatrixXf> &pFeatures);

    ~Particle();

    float &w();

    /// Robot pose: x, y, theta (heading direction)
    VectorXf &xv();
    /// Controls: velocities, vehicle pose predict covariance
    MatrixXf &Pv();
    /// 2D means of landmark EKFs
    vector<VectorXf> &landmarkXs();
    /// Covariance matrices for landmark EKFs
    vector<MatrixXf> &landmarkPs();

    //setters
    void setW(float w);
    void setXv(VectorXf &xv);
    void setPv(MatrixXf &Pv);
    void setLandmarkX(unsigned long i, VectorXf &vec);
    void setLandmarkP(unsigned long i, MatrixXf &m);

private:
    float _w;
    VectorXf _xv;
    MatrixXf _Pv;
    vector<VectorXf> _landmarkXs;
    vector<MatrixXf> _landmarkPs;
};

#endif //SLAM_GUI_PARTICLE_H
