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

    Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &xf, vector<MatrixXf> &Pf, float *da);

    ~Particle();

    //getters
    float &w();

    VectorXf &xv();             //robot pose: x,y,theta (heading dir)
    MatrixXf &Pv();             //controls: velocities
    vector<VectorXf> &xf();     //2d means of EKF
    vector<MatrixXf> &Pf();     //covariance matrices for EKF
    float *da();

    //setters
    void setW(float w);

    void setXv(VectorXf &xv);

    void setPv(MatrixXf &Pv);

    void setXf(vector<VectorXf> &xf);

    void setXfi(unsigned long i, VectorXf &vec);

    void setPf(vector<MatrixXf> &Pf);

    void setPfi(unsigned long i, MatrixXf &m);

    void setDa(float *da);

private:
    float _w;
    VectorXf _xv;
    MatrixXf _Pv;
    vector<VectorXf> _xf;
    vector<MatrixXf> _Pf;
    float *_da;
};

#endif //SLAM_GUI_PARTICLE_H
