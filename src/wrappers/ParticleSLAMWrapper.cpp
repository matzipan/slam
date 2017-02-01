//
// Created by matzipan on 30/01/17.
//
#include <QtGui>
#include <src/core.h>
#include <src/plot.h>

#include "ParticleSLAMWrapper.h"

ParticleSLAMWrapper::ParticleSLAMWrapper(Conf *conf, Plot *plot, QObject *parent) : SLAMWrapper(conf, plot, parent) {
    particles = vector<Particle>(conf->NPARTICLES);
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

void ParticleSLAMWrapper::initializeDataAssociationTable() {
    dataAssociationTable = VectorXf(landmarks.cols());
    for (int s = 0; s < dataAssociationTable.size(); s++) {
        dataAssociationTable[s] = -1;
    }
}

void ParticleSLAMWrapper::drawParticles() {
    QVector<double> particleXs, particleYs;

    for (int i = 0; i < particles.size(); i++) {
        particleXs.push_back(particles[i].xv()(0));
        particleYs.push_back(particles[i].xv()(1));
    }
    plot->setParticles(particleXs, particleYs);
}

void ParticleSLAMWrapper::drawFeatureParticles() {
    QVector<double> featureParticleXs, featureParticleYs;

    for (int i = 0; i < particles.size(); i++) {
        for (unsigned long j = 0; j < particles[i].xf().size(); j++) {
            featureParticleXs.push_back(particles[i].xf()[j](0));
            featureParticleYs.push_back(particles[i].xf()[j](1));
        }
    }
    plot->setParticlesFea(featureParticleXs, featureParticleYs);
}

void ParticleSLAMWrapper::computeEstimatedPosition(double &x, double &y, double &t) {
    // Compute xEstimated and y as the mean of xEstimated and y over all the particles and the t is the angle corresponding to the particle
    // with the highest weight.
    double wMax;

    x = 0;
    y = 0;
    t = 0;

    wMax = -1e30;
    for (int i = 0; i < particles.size(); i++) {
        if (particles[i].w() > wMax) {
            wMax = particles[i].w();
            t = particles[i].xv()(2);
        }
        x += particles[i].xv()(0);
        y += particles[i].xv()(1);
    }

    x = x / particles.size();
    y = y / particles.size();
}
