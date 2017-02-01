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

    QVector<double> arrParticles_x, arrParticles_y;
    QVector<double> arrParticlesFea_x, arrParticlesFea_y;

    double w_max;
    double x_mean, y_mean, t_max;

    // Main loop
    while (isAlive) {
        int controlStatus = control();

        if(controlStatus == -1) {
            break;
        }

        if(controlStatus == 0) {
            continue;
        }


        // @TODO what happens when the if statement is false and the predict happens but not the observation
        // @TODO why does fastslam1 use Q and fastslam 2 use Qe
        algorithm->predict(particles, xTrue, Vn, Gn, Qe, conf->WHEELBASE, dt, conf->SWITCH_PREDICT_NOISE == 1, conf->SWITCH_HEADING_KNOWN == 1);

        dtSum += dt;
        bool observe = false;

        if (dtSum >= conf->DT_OBSERVE) {
            observe = true;
            dtSum = 0;

            vector<int> ftag_visible = vector<int>(landmarkIdentifiers); // Modify the copy, not the landmarkIdentifiers

            // Compute true data, then add noise
            landmarksRangeBearing = getObservations(landmarks, xTrue, ftag_visible, conf->MAX_RANGE);
            if(conf->SWITCH_SENSOR_NOISE) {
                addObservationNoise(landmarksRangeBearing, R);
            }

            plines = makeLaserLines(landmarksRangeBearing, xTrue);

            // Compute (known) data associations
            unsigned long Nf = particles[0].xf().size();
            vector<int> idf;
            vector<VectorXf> zf;
            vector<VectorXf> zn;

            data_associate_known(landmarksRangeBearing, ftag_visible, dataAssociationTable, Nf, zf, idf, zn);

            // @TODO why does fastslam1 use R and fastslam 2 use Re
            algorithm->update(particles, zf, zn, idf, landmarksRangeBearing, ftag_visible, dataAssociationTable, Re, conf->NEFFECTIVE, conf->SWITCH_RESAMPLE == 1);
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

    delete[] VnGn; //@TODO is this really needed? Is it a memory leak?
}
