#include <stdlib.h>

#include <QtGui>
#include <src/core.h>

#include "slamwrapper.h"
#include "src/plot.h"

#include "src/utils.h"

#include "moc_slamwrapper.cpp"

extern Plot *gPlot;
extern Conf *gConf;

SLAMWrapper::SLAMWrapper(QObject *parent) : QThread(parent) {
    isAlive = 1;

    commandId = -1;
    commandTime = 0;

    Q = MatrixXf(2, 2);
    Q << pow(gConf->sigmaV, 2), 0, 0, pow(gConf->sigmaG, 2);

    R = MatrixXf(2, 2);
    R << pow(gConf->sigmaR, 2), 0, 0, pow(gConf->sigmaB, 2);

    if (gConf->SWITCH_INFLATE_NOISE == 1) {
        Qe = 2 * Q;
        Re = 2 * R;
    } else {
        Qe = MatrixXf(Q);
        Re = MatrixXf(R);
    }

    sigmaPhi = gConf->sigmaT;

    // Setting initial velocity
    V = gConf->V;
    // Setting initial steer angle
    G = 0;
    nLoop = gConf->NUMBER_LOOPS;
    dt = gConf->DT_CONTROLS;

    xTrue = VectorXf(3);
    xTrue.setZero(3);

    x = VectorXf(3);
    x.setZero(3);

    drawSkip = 4;
    gConf->i("drawSkip", drawSkip);

    qRegisterMetaType<QString>("QString");

    connect(this, SIGNAL(replot()), gPlot, SLOT(canvasReplot()));
    connect(this, SIGNAL(showMessage(QString)), gPlot, SLOT(canvasShowMessage(QString)));

    connect(gPlot, SIGNAL(commandSend(int)), this, SLOT(commandRecieve(int)));

    if (gConf->SWITCH_SEED_RANDOM != 0) {
        srand(gConf->SWITCH_SEED_RANDOM);
    }
}

SLAMWrapper::~SLAMWrapper() { }

void SLAMWrapper::stop() {
    isAlive = 0;
}

void SLAMWrapper::commandRecieve(int command) {
    commandId = command;
    commandTime = tm_get_millis();
}

int SLAMWrapper::getCurrentCommand() {
    u_int64_t timeNow;

    timeNow = tm_get_millis();

    if (timeNow - commandTime < 30) {
        return commandId;
    } else {
        return -1;
    }
}

void SLAMWrapper::setRunMode(RunMode mode) {
    runMode = mode;
}

void SLAMWrapper::loadMap(string &filename) {
    map = filename;

    read_slam_input_file(map, &landmarks, &waypoints);
}

void SLAMWrapper::initializeLandmarkIdentifiers() {
    for (int i = 0; i < landmarks.cols(); i++) {
        ftag.push_back(i);
    }
}


void SLAMWrapper::configurePlot() {
    gPlot->setCarSize(gConf->WHEELBASE, 0);
    gPlot->setCarSize(gConf->WHEELBASE, 1);

    addWaypointsAndLandmarks();
    setPlotRange();

    // Add initial position
    gPlot->addTruePosition(xTrue(0), xTrue(1));
    gPlot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));

    // Add initial position estimate
    gPlot->addEstimatedPosition(xTrue(0), xTrue(1));
    gPlot->setCarEstimatedPosition(xTrue(0), xTrue(1), xTrue(2));

    emit replot();
}

void SLAMWrapper::addWaypointsAndLandmarks() {
    uint n;

    QVector<double> waypointXs, waypointYs;
    QVector<double> landmarkXs, landmarkYs;

    // draw waypoints
    if (runMode == SLAM_WAYPOINT) {
        n = waypoints.cols();

        for (uint i = 0; i < n; i++) {
            waypointXs.push_back(waypoints(0, i));
            waypointYs.push_back(waypoints(1, i));
        }

        gPlot->setWaypoints(waypointXs, waypointYs);
    }

    // Draw landmarks
    n = landmarks.cols();
    for (uint i = 0; i < n; i++) {
        landmarkXs.push_back(landmarks(0, i));
        landmarkYs.push_back(landmarks(1, i));
    }

    gPlot->setLandmarks(landmarkXs, landmarkYs);

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

    gPlot->setPlotRange(xMin - (xMax - xMin) * 0.05, xMax + (xMax - xMin) * 0.05,
                        yMin - (yMax - yMin) * 0.05, yMax + (yMax - yMin) * 0.05);

}

int SLAMWrapper::control() {
    switch(runMode) {
        case SLAM_WAYPOINT:
            if (indexOfFirstWaypoint == -1) {
                return -1;
            }

            compute_steering(xTrue, waypoints, indexOfFirstWaypoint, gConf->AT_WAYPOINT, G, gConf->RATEG, gConf->MAXG, dt);

            if (indexOfFirstWaypoint == -1 && nLoop > 1) {
                indexOfFirstWaypoint = 0;
                nLoop--;
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
                    V = gConf->V;
                    G = 0.0;
                    break;
                case 2:
                    // Backward
                    V = -gConf->V;
                    G = 0.0;
                    break;
                case 3:
                    // Turn left
                    V = gConf->V;
                    G = 30.0 * M_PI / 180.0;
                    break;
                case 4:
                    // Turn right
                    V = gConf->V;
                    G = -30.0 * M_PI / 180.0;
                    break;
                default:
                    V = gConf->V;
                    G = 0.0;
            }
            break;
    }

    // Predict current position and angle
    predict_true(xTrue, V, G, gConf->WHEELBASE, dt);

    // Add process noise
    add_control_noise(V, G, Q, gConf->SWITCH_CONTROL_NOISE, VnGn);
    Vn = VnGn[0];
    Gn = VnGn[1];

    return 1;
}