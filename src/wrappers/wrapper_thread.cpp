#include <stdlib.h>

#include <QtGui>

#include "wrapper_thread.h"
#include "src/plot.h"

#include "src/utils.h"

#include "moc_wrapper_thread.cpp"

// global variable
extern Plot *g_plot;


Wrapper_Thread::Wrapper_Thread(QObject *parent) : QThread(parent) {
    isAlive = 1;

    commandID = -1;
    commandTime = 0;

    qRegisterMetaType<QString>("QString");

    connect(this, SIGNAL(replot()), g_plot, SLOT(canvasReplot()));
    connect(this, SIGNAL(showMessage(QString)), g_plot, SLOT(canvasShowMessage(QString)));

    connect(g_plot, SIGNAL(commandSend(int)), this, SLOT(commandRecv(int)));
}

Wrapper_Thread::~Wrapper_Thread() {
    wait();
}

void Wrapper_Thread::stop(void) {
    isAlive = 0;
}

void Wrapper_Thread::commandRecv(int cmd) {
    commandID = cmd;
    commandTime = tm_get_millis();
}

void Wrapper_Thread::getCommand(int *cmd) {
    u_int64_t time_now, dt;

    time_now = tm_get_millis();
    dt = time_now - commandTime;

    if (dt < 30) {
        *cmd = commandID;
    } else {
        *cmd = -1;
    }
}

// set run mode
void Wrapper_Thread::setRunMode(RunMode mode) {
    runMode = mode;
}

// set map filename
void Wrapper_Thread::setMap(std::string &fname) {
    fnMap = fname;
}
