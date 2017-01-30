//
// Created by matzipan on 30/01/17.
//
#include <QtGui>
#include <src/core.h>

#include "ParticleSLAMWrapper.h"

extern Conf *gConf;

ParticleSLAMWrapper::ParticleSLAMWrapper(QObject *parent) : SLAMWrapper(parent) {
    particles = vector<Particle>(gConf->NPARTICLES);
}

ParticleSLAMWrapper::~ParticleSLAMWrapper() { }

void ParticleSLAMWrapper::initializeParticles() {
    // Vector of particles (their count will change)
    for (unsigned long i = 0; i < particles.size(); i++) {
        particles[i] = Particle();
    }

    // Initialize particle weights as uniform
    float uniformw = 1.0 / (float) particles.size();
    for (int p = 0; p < particles.size(); p++) {
        particles[p].setW(uniformw);
    }
}