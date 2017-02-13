#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include <Eigen/Dense>

#include "src/backend/core.h"
#include "src/backend/algorithms/ekfslam.h"
#include "ekfslamwrapper.h"

using namespace std;
using namespace Eigen;


EKFSLAMWrapper::EKFSLAMWrapper(Conf *conf, NetworkPlot *plot, QObject *parent) : SLAMWrapper(conf, plot, parent) {
    algorithm = new EKFSLAM();

    algorithm->enableBatchUpdate = conf->SWITCH_BATCH_UPDATE == 1;
    algorithm->useHeading = conf->SWITCH_HEADING_KNOWN == 1;
    algorithm->wheelBase = conf->WHEELBASE;
    algorithm->gateReject = conf->GATE_REJECT;
    algorithm->gateAugment = conf->GATE_AUGMENT;
    algorithm->associationKnown = conf->SWITCH_ASSOCIATION_KNOWN;
    algorithm->sigmaPhi = conf->sigmaT;

}

EKFSLAMWrapper::~EKFSLAMWrapper() {
    wait();
}


void EKFSLAMWrapper::run() {
    printf("EKFSLAM\n\n");

    configurePlot();
    initializeLandmarkIdentifiers();
    initializeDataAssociationTable();

    P = MatrixXf(3, 3);
    P.setZero(3, 3);

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

        dtSum += dt;
        bool observe = false;

        if (dtSum >= conf->DT_OBSERVE) {
            observe = true;
            dtSum = 0;

            // Modify the copy, not the landmarkIdentifiers
            vector<int> visibleLandmarkIdentifiers = vector<int>(landmarkIdentifiers);

            // Compute true data, then add noise
            landmarksRangeBearing = getObservations(landmarks, xTrue, visibleLandmarkIdentifiers, conf->MAX_RANGE);
            if(conf->SWITCH_SENSOR_NOISE) {
                addObservationNoise(landmarksRangeBearing, R);
            }

            plines = makeLaserLines(landmarksRangeBearing, xTrue);
        }

        algorithm->sim(landmarks, waypoints, xEstimated, P, Vnoisy, Gnoisy, Qe, dt,
                       xTrue(2) + conf->sigmaT * unifRand(), landmarkIdentifiers,
                       landmarksRangeBearing, Re, observe, zf, idf, zn, dataAssociationTable,
                       R);

        // Update status bar
        currentIteration++;

        // Accelate drawing speed
        if (currentIteration % drawSkip != 0) {
            continue;
        }

        emit setCurrentIteration(currentIteration);

        // Add new position
        plot->addTruePosition(xTrue(0), xTrue(1));
        plot->addEstimatedPosition(xEstimated(0), xEstimated(1));

        // Draw current position
        plot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));
        plot->setCarEstimatedPosition(xEstimated(0), xEstimated(1), xEstimated(2));

        // Set laser lines
        plot->setLaserLines(plines);

        drawCovarianceEllipseLines();

        emit replot();
    }
}

void EKFSLAMWrapper::initializeDataAssociationTable() {
    for (int i = 0; i < landmarks.cols(); i++) {
        dataAssociationTable.push_back(-1);
    }
}

void EKFSLAMWrapper::drawCovarianceEllipseLines() {
    MatrixXf covarianceEllipseLines;

    MatrixXf x_(2, 1);
    MatrixXf P_ = P.block(0, 0, 2, 2);
    x_(0) = xEstimated(0);
    x_(1) = xEstimated(1);

    makeCovarianceEllipse(x_, P_, covarianceEllipseLines);
    plot->setCovEllipse(covarianceEllipseLines, 0);

    int j = (xEstimated.size() - 3) / 2;
    for (int i = 0; i < j; i++) {
        x_(0) = xEstimated(3 + i * 2);
        x_(1) = xEstimated(3 + i * 2 + 1);
        P_ = P.block(3 + i * 2, 3 + i * 2, 2, 2);

        makeCovarianceEllipse(x_, P_, covarianceEllipseLines);
        plot->setCovEllipse(covarianceEllipseLines, i + 1);
    }
}