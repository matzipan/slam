#include <stdlib.h>

#include <QtGui>

#include "slamwrapper.h"
#include "src/plot.h"

#include "src/utils.h"

#include "moc_slamwrapper.cpp"

extern Plot *gPlot;

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
