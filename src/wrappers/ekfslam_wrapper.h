#ifndef __EKFSLAM_WRAPPER_H__
#define __EKFSLAM_WRAPPER_H__

#include <QtGui>
#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/algorithms/ekfslam.h"
#include "wrapper_thread.h"

using namespace std;
using namespace Eigen;


class EKFSLAM_Wrapper : public Wrapper_Thread {
    Q_OBJECT

public:
    EKFSLAM_Wrapper(QObject *parent = 0);
    ~EKFSLAM_Wrapper();

protected:
    void run();
    EKFSLAM *algorithm;
};

#endif // __EKFSLAM_WRAPPER_H__
