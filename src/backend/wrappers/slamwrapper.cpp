#include <stdlib.h>

#include <QtGui>
#include <src/backend/core.h>

#include "slamwrapper.h"

SLAMWrapper::SLAMWrapper(Conf *conf, NetworkPlot *plot, QObject *parent) : QThread(parent) {
    this->conf = conf;
    this->plot = plot;

    if (conf->s("mode") == "interactive") { setRunMode(SLAMWrapper::SLAM_INTERACTIVE); }
    else { setRunMode(SLAMWrapper::SLAM_WAYPOINT); }

    loadMap();

    commandId = -1;
    commandTime = 0;

    // Setting initial velocity
    Vtrue = conf->V;
    // Setting initial steer angle
    Gtrue = 0;

    Q = MatrixXf(2, 2);
    Q << pow(conf->sigmaV, 2), 0, 0, pow(conf->sigmaG, 2);

    R = MatrixXf(2, 2);
    R << pow(conf->sigmaR, 2), 0, 0, pow(conf->sigmaB, 2);

    if (conf->SWITCH_INFLATE_NOISE == 1) {
        Q = 2 * Q;
        R = 2 * R;
    } else {
        Qe = MatrixXf(Q);
        Re = MatrixXf(R);
    }

    nLoop = conf->NUMBER_LOOPS;
    dt = conf->DT_CONTROLS;

    xTrue = VectorXf(3);
    xTrue.setZero(3);

    xEstimated = VectorXf(3);
    xEstimated.setZero(3);

    drawSkip = 4;
    conf->i("drawSkip", drawSkip);

    connect(this, SIGNAL(replot()), this->plot, SLOT(plot()));
    connect(this, SIGNAL(setCurrentIteration(int)), this->plot, SLOT(setCurrentIteration(int)));

    connect(this->plot, SIGNAL(commandSend(int)), this, SLOT(commandRecieve(int)));

    if (conf->SWITCH_SEED_RANDOM != 0) {
        srand(conf->SWITCH_SEED_RANDOM);
    }
}

SLAMWrapper::~SLAMWrapper() { }

void SLAMWrapper::stop() {
    isAlive = false;
}

void SLAMWrapper::commandRecieve(int command) {
    commandId = command;
    commandTime = getMillisecondsTime();
}

int SLAMWrapper::getCurrentCommand() {
    unsigned long timeNow;

    timeNow = getMillisecondsTime();

    if (timeNow - commandTime < 30) {
        return commandId;
    } else {
        return -1;
    }
}

void SLAMWrapper::setRunMode(RunMode mode) {
    runMode = mode;
}

void SLAMWrapper::loadMap() {
    string map = conf->s("m");

    readInputFile(map, &landmarks, &waypoints);
}

void SLAMWrapper::initializeLandmarkIdentifiers() {
    for (int i = 0; i < landmarks.cols(); i++) {
        landmarkIdentifiers.push_back(i);
    }
}


void SLAMWrapper::configurePlot() {
    plot->setCarSize(conf->WHEELBASE, 0);
    plot->setCarSize(conf->WHEELBASE, 1);

    addWaypointsAndLandmarks();
    setPlotRange();

    // Add initial position
    plot->addTruePosition(xTrue(0), xTrue(1));
    plot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));

    // Add initial position estimate
    plot->addEstimatedPosition(xTrue(0), xTrue(1));
    plot->setCarEstimatedPosition(xTrue(0), xTrue(1), xTrue(2));

    plot->plot();
}

void SLAMWrapper::addWaypointsAndLandmarks() {
    uint n;

    QVector<double> waypointXs, waypointYs;
    QVector<double> landmarkXs, landmarkYs;

    // Draw waypoints
    if (runMode == SLAM_WAYPOINT) {
        n = waypoints.cols();

        for (uint i = 0; i < n; i++) {
            waypointXs.push_back(waypoints(0, i));
            waypointYs.push_back(waypoints(1, i));
        }

        plot->setWaypoints(waypointXs, waypointYs);
    }

    // Draw landmarks
    n = landmarks.cols();
    for (uint i = 0; i < n; i++) {
        landmarkXs.push_back(landmarks(0, i));
        landmarkYs.push_back(landmarks(1, i));
    }

    plot->setLandmarks(landmarkXs, landmarkYs);

}

void SLAMWrapper::setPlotRange() {
    uint n;
    double xMin = 1e30;
    double xMax = -1e30;
    double yMin = 1e30;
    double yMax = -1e30;

    // Find waypoints range
    if (runMode == SLAM_WAYPOINT) {
        n = waypoints.cols();

        for (uint i = 0; i < n; i++) {
            if (waypoints(0, i) > xMax) { xMax = waypoints(0, i); }
            if (waypoints(0, i) < xMin) { xMin = waypoints(0, i); }
            if (waypoints(1, i) > yMax) { yMax = waypoints(1, i); }
            if (waypoints(1, i) < yMin) { yMin = waypoints(1, i); }
        }
    }

    // Find landmarks range
    n = landmarks.cols();
    for (uint i = 0; i < n; i++) {
        if (landmarks(0, i) > xMax) { xMax = landmarks(0, i); }
        if (landmarks(0, i) < xMin) { xMin = landmarks(0, i); }
        if (landmarks(1, i) > yMax) { yMax = landmarks(1, i); }
        if (landmarks(1, i) < yMin) { yMin = landmarks(1, i); }
    }

    plot->setPlotRange(xMin - (xMax - xMin) * 0.05, xMax + (xMax - xMin) * 0.05,
                        yMin - (yMax - yMin) * 0.05, yMax + (yMax - yMin) * 0.05);

}

int SLAMWrapper::control() {
    switch(runMode) {
        case SLAM_WAYPOINT:
            if (indexOfFirstWaypoint == -1) {
                return -1;
            }

            updateSteering(xTrue, waypoints, indexOfFirstWaypoint, conf->AT_WAYPOINT, Gtrue, conf->RATEG, conf->MAXG, dt);

            if (indexOfFirstWaypoint == -1 && nLoop > 1) {
                indexOfFirstWaypoint = 0;
                nLoop--;
            }

            if(indexOfFirstWaypoint == -1 && nLoop == 1) {
                stop();
                jobFinished();
            }
            break;

        case SLAM_INTERACTIVE:
            int command = getCurrentCommand();

            // No commands, then continue
            if (command == -1) {
                return 0;
            }

            switch (command) {
                case 1:
                    // Forward
                    Vtrue = conf->V;
                    Gtrue = 0.0;
                    break;
                case 2:
                    // Backward
                    Vtrue = -conf->V;
                    Gtrue = 0.0;
                    break;
                case 3:
                    // Turn left
                    Vtrue = conf->V;
                    Gtrue = 30.0 * M_PI / 180.0;
                    break;
                case 4:
                    // Turn right
                    Vtrue = conf->V;
                    Gtrue = -30.0 * M_PI / 180.0;
                    break;
                default:
                    Vtrue = conf->V;
                    Gtrue = 0.0;
            }
            break;
    }

    // Predict current position and angle
    predictTruePosition(xTrue, Vtrue, Gtrue, conf->WHEELBASE, dt);

    if(conf->SWITCH_CONTROL_NOISE) {
        // Add process noise
        addControlNoise(Vtrue, Gtrue, Q, Vnoisy, Gnoisy);
    }

    return 1;
}