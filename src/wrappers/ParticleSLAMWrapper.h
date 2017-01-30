//
// Created by matzipan on 30/01/17.
//

#ifndef SLAM_GUI_PARTICLESLAMWRAPPER_H
#define SLAM_GUI_PARTICLESLAMWRAPPER_H

#include <QtGui>
#include <src/particle.h>
#include "slamwrapper.h"

class ParticleSLAMWrapper : public SLAMWrapper {
Q_OBJECT

public:
    ParticleSLAMWrapper(QObject *parent = 0);

    ~ParticleSLAMWrapper();

protected:
    void initializeParticles();

    vector<Particle> particles;

};

#endif //SLAM_GUI_PARTICLESLAMWRAPPER_H
