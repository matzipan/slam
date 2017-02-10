#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iterator>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/plotting/WindowPlot.h"
#include "src/Particle.h"
#include "src/algorithms/fastslam1.h"
#include "fastslam1wrapper.h"

using namespace std;
using namespace Eigen;

FastSLAM1Wrapper::FastSLAM1Wrapper(Conf *conf, NetworkPlot *plot, QObject *parent) : ParticleSLAMWrapper(conf, plot, parent) {
    algorithm = new FastSLAM1();

    algorithm->addPredictNoise = 1; // Always use predict noise
    algorithm->useHeading = conf->SWITCH_HEADING_KNOWN == 1;
    algorithm->wheelBase = conf->WHEELBASE;
    algorithm->nEffective = conf->NEFFECTIVE;
    algorithm->resample = conf->SWITCH_RESAMPLE == 1;
}

FastSLAM1Wrapper::~FastSLAM1Wrapper() {
    wait();
}

void FastSLAM1Wrapper::run() {
    printf("FastSLAM 1\n\n");

    configurePlot();
    initializeParticles();
    initializeLandmarkIdentifiers();
    initializeDataAssociationTable();

    vector<int> idf;
    vector<VectorXf> zf;
    vector<VectorXf> zn;

    // Main loop
    while (isAlive) {
        int controlStatus = control();

        if(controlStatus == -1) {
            break;
        }

        if(controlStatus == 0) {
            continue;
        }

        algorithm->predict(particles, xTrue, Vnoisy, Gnoisy, Qe, dt);

        dtSum += dt;

        if (dtSum >= conf->DT_OBSERVE) {
            dtSum = 0;

            vector<int> visibleLandmarkIdentifiers = vector<int>(landmarkIdentifiers); // Modify the copy, not the landmarkIdentifiers

            // Compute true data, then add noise
            landmarksRangeBearing = getObservations(landmarks, xTrue, visibleLandmarkIdentifiers, conf->MAX_RANGE);
            if(conf->SWITCH_SENSOR_NOISE) {
                addObservationNoise(landmarksRangeBearing, R);
            }

            plines = makeLaserLines(landmarksRangeBearing, xTrue);

            // Compute (known) data associations
            unsigned long Nf = particles[0].landmarkXs().size();

            dataAssociationKnown(landmarksRangeBearing, visibleLandmarkIdentifiers, dataAssociationTable, Nf, zf, idf,
                                 zn);

            algorithm->update(particles, zf, zn, idf, visibleLandmarkIdentifiers, dataAssociationTable, Re);
        }


        // Update status bar
        currentIteration++;

        // Accelate drawing speed
        if (currentIteration % drawSkip != 0) {
            continue;
        }

        emit setCurrentIteration(currentIteration);

        drawParticles();
        drawFeatureParticles();

        double x, y, t;
        computeEstimatedPosition(x, y, t);

        // Add new position
        plot->addTruePosition(xTrue(0), xTrue(1));
        plot->addEstimatedPosition(x, y);

        // Draw current position
        plot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));
        plot->setCarEstimatedPosition(x, y, t);

        // Set laser lines
        plot->setLaserLines(plines);

        emit replot();
    }
}