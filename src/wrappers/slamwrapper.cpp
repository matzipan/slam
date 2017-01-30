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

    qRegisterMetaType<QString>("QString");

    connect(this, SIGNAL(replot()), gPlot, SLOT(canvasReplot()));
    connect(this, SIGNAL(showMessage(QString)), gPlot, SLOT(canvasShowMessage(QString)));

    connect(gPlot, SIGNAL(commandSend(int)), this, SLOT(commandRecieve(int)));
}

SLAMWrapper::~SLAMWrapper() {
    wait();
}

void SLAMWrapper::stop() {
    isAlive = 0;
}

void SLAMWrapper::commandRecieve(int command) {
    commandId = command;
    commandTime = tm_get_millis();
}

void SLAMWrapper::getCommand(int *command) {
    u_int64_t timeNow, dt;

    timeNow = tm_get_millis();
    dt = timeNow - commandTime;

    if (dt < 30) {
        *command = commandId;
    } else {
        *command = -1;
    }
}

void SLAMWrapper::setRunMode(RunMode mode) {
    runMode = mode;
}

void SLAMWrapper::setMap(std::string &fname) {
    map = fname;
}

void SLAMWrapper::configurePlot() {
    gPlot->setCarSize(gConf->WHEELBASE, 0);
    gPlot->setCarSize(gConf->WHEELBASE, 1);

    addWaypointsAndLandmarks();
    setPlotRange();
}

void SLAMWrapper::addWaypointsAndLandmarks() {
    uint m, n;

    QVector<double> arrWaypoints_x, arrWaypoints_y;
    QVector<double> arrLandmarks_x, arrLandmarks_y;

    // draw waypoints
    if (runMode == SLAM_WAYPOINT) {
        n = waypoints.cols();

        for (int i = 0; i < n; i++) {
            arrWaypoints_x.push_back(waypoints(0, i));
            arrWaypoints_y.push_back(waypoints(1, i));
        }

        gPlot->setWaypoints(arrWaypoints_x, arrWaypoints_y);
    }

    // draw landmarks
    n = landmarks.cols();
    for (int i = 0; i < n; i++) {
        arrLandmarks_x.push_back(landmarks(0, i));
        arrLandmarks_y.push_back(landmarks(1, i));
    }

    gPlot->setLandmarks(arrLandmarks_x, arrLandmarks_y);

}

void SLAMWrapper::setPlotRange() {
    uint m, n;
    double x_min = 1e30;
    double x_max = -1e30;
    double y_min = 1e30;
    double y_max = -1e30;

    // Draw waypoints
    if (runMode == SLAM_WAYPOINT) {
        m = waypoints.rows();
        n = waypoints.cols();

        for (int i = 0; i < n; i++) {
            if (waypoints(0, i) > x_max) { x_max = waypoints(0, i); }
            if (waypoints(0, i) < x_min) { x_min = waypoints(0, i); }
            if (waypoints(1, i) > y_max) { y_max = waypoints(1, i); }
            if (waypoints(1, i) < y_min) { y_min = waypoints(1, i); }
        }
    }

    // draw landmarks
    m = landmarks.rows();
    n = landmarks.cols();
    for (int i = 0; i < n; i++) {
        if (landmarks(0, i) > x_max) { x_max = landmarks(0, i); }
        if (landmarks(0, i) < x_min) { x_min = landmarks(0, i); }
        if (landmarks(1, i) > y_max) { y_max = landmarks(1, i); }
        if (landmarks(1, i) < y_min) { y_min = landmarks(1, i); }
    }

    gPlot->setPlotRange(x_min - (x_max - x_min) * 0.05, x_max + (x_max - x_min) * 0.05,
                        y_min - (y_max - y_min) * 0.05, y_max + (y_max - y_min) * 0.05);

}