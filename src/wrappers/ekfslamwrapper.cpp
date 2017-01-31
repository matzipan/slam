#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/plot.h"
#include "src/algorithms/ekfslam.h"
#include "ekfslamwrapper.h"

using namespace std;
using namespace Eigen;


EKFSLAMWrapper::EKFSLAMWrapper(Conf *conf, Plot *plot, QObject *parent) : SLAMWrapper(conf, plot, parent) {
    algorithm = new EKFSLAM();
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

            //Compute true data, then add noise
            vector<int> ftag_visible = vector<int>(ftag); // Modify the copy, not the ftag

            //z is the range and bearing of the observed landmark
            z = get_observations(xTrue, landmarks, ftag_visible, conf->MAX_RANGE);
            add_observation_noise(z, R, conf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xTrue);
        }

        // @TODO what happens if this is moved inside the if(OBSERVE) branch?
        algorithm->sim(landmarks, waypoints, x, P, Vn, Gn, Qe,
                       conf->WHEELBASE, dt, xTrue(2) + conf->sigmaT * unifRand(), conf->SWITCH_HEADING_KNOWN,
                       sigmaPhi, ftag,
                       z, Re, conf->GATE_REJECT, conf->GATE_AUGMENT, conf->SWITCH_ASSOCIATION_KNOWN, observe, zf,
                       idf, zn,
                       dataAssociationTable, conf->SWITCH_BATCH_UPDATE == 1, R);

        // Update status bar
        currentIteration++;

        // Accelate drawing speed
        if (currentIteration % drawSkip != 0) {
            continue;
        }

        plotMessage.sprintf("[%6d]", currentIteration);
        emit showMessage(plotMessage);

        // Add new position
        plot->addTruePosition(xTrue(0), xTrue(1));
        plot->addEstimatedPosition(x(0), x(1));

        // Draw current position
        plot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));
        plot->setCarEstimatedPosition(x(0), x(1), x(2));

        // Set laser lines
        plot->setLaserLines(plines);

        drawCovarianceEllipseLines();

        emit replot();
    }

    delete[] VnGn; //@TODO is this really needed? Is it a memory leak?
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
    x_(0) = x(0);
    x_(1) = x(1);

    make_covariance_ellipse(x_, P_, covarianceEllipseLines);
    plot->setCovEllipse(covarianceEllipseLines, 0);

    int j = (x.size() - 3) / 2;
    for (int i = 0; i < j; i++) {
        x_(0) = x(3 + i * 2);
        x_(1) = x(3 + i * 2 + 1);
        P_ = P.block(3 + i * 2, 3 + i * 2, 2, 2);

        make_covariance_ellipse(x_, P_, covarianceEllipseLines);
        plot->setCovEllipse(covarianceEllipseLines, i + 1);
    }
}