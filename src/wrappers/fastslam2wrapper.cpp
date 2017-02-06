#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iterator>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/plot.h"
#include "src/particle.h"
#include "src/algorithms/fastslam2.h"
#include "fastslam2wrapper.h"

using namespace std;
using namespace Eigen;

FastSLAM2Wrapper::FastSLAM2Wrapper(Conf *conf, Plot *plot, QObject *parent) : ParticleSLAMWrapper(conf, plot, parent) {
    algorithm = new FastSLAM2();

    algorithm->addPredictNoise = conf->SWITCH_PREDICT_NOISE == 1;
    algorithm->useHeading = conf->SWITCH_HEADING_KNOWN == 1;
    algorithm->wheelBase = conf->WHEELBASE;
    algorithm->nEffective = conf->NEFFECTIVE;
    algorithm->resample = conf->SWITCH_RESAMPLE == 1;
}

FastSLAM2Wrapper::~FastSLAM2Wrapper() {
    wait();
}


void FastSLAM2Wrapper::run() {
    printf("FastSLAM 2\n\n");

    if (conf->SWITCH_PREDICT_NOISE) {
        printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");
    }
    if (conf->SWITCH_SAMPLE_PROPOSAL == 0) {
        printf("Sampling from optimal proposal is usually ON for FastSLAM 2.0\n");
    }

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
        bool observe = false;

        if (dtSum >= conf->DT_OBSERVE) {
            observe = true;
            dtSum = 0;

            vector<int> visibleLandmarkIdentifiers = vector<int>(landmarkIdentifiers); // Modify the copy, not the landmarkIdentifiers

            // Compute true data, then add noise
            landmarksRangeBearing = getObservations(landmarks, xTrue, visibleLandmarkIdentifiers, conf->MAX_RANGE);
            if(conf->SWITCH_SENSOR_NOISE) {
                addObservationNoise(landmarksRangeBearing, R);
            }

            plines = makeLaserLines(landmarksRangeBearing, xTrue);

            // Compute (known) data associations
            unsigned long Nf = particles[0].xf().size();

            dataAssociationKnown(landmarksRangeBearing, visibleLandmarkIdentifiers, dataAssociationTable, Nf, zf, idf, zn);

            algorithm->update(particles, zf, zn, idf, landmarksRangeBearing, dataAssociationTable,
                              Re);
        }


        // Update status bar
        currentIteration++;

        // Accelate drawing speed
        if (currentIteration % drawSkip != 0) {
            continue;
        }

        plotMessage.sprintf("[%6d]", currentIteration);
        emit showMessage(plotMessage);

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
