//
// Created by matzipan on 11/11/16.
//

#include "particle.h"

////////////////////////////////////////////////////////////////////////////////
// particle class
////////////////////////////////////////////////////////////////////////////////
Particle::Particle() {
    _w = 1.0;
    _xv = VectorXf(3);
    _xv.setZero(3);
    _Pv = MatrixXf(3, 3);
    _Pv.setZero(3, 3);
}

Particle::Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &landmarkXs, vector<MatrixXf> &landmarkPs) {
    _w = w;
    _xv = xv;
    _Pv = Pv;
    _landmarkXs = landmarkXs;
    _landmarkPs = landmarkPs;
}

Particle::~Particle() {
}

float &Particle::w() {
    return _w;
}

VectorXf &Particle::xv() {
    return _xv;
}

MatrixXf &Particle::Pv() {
    return _Pv;
}

vector<VectorXf> &Particle::landmarkXs() {
    return _landmarkXs;
}

vector<MatrixXf> &Particle::landmarkPs() {
    return _landmarkPs;
}

void Particle::setW(float w) {
    _w = w;
}

void Particle::setXv(VectorXf &xv) {
    _xv = xv;
}

void Particle::setPv(MatrixXf &Pv) {
    _Pv = Pv;
}

void Particle::setLandmarkX(unsigned long i, VectorXf &vec) {
    if (i >= _landmarkXs.size()) {
        _landmarkXs.resize(i + 1);
    }
    _landmarkXs[i] = vec;
}

void Particle::setLandmarkP(unsigned long i, MatrixXf &m) {
    if (i >= _landmarkPs.size()) {
        _landmarkPs.resize(i + 1);
    }
    _landmarkPs[i] = m;
}