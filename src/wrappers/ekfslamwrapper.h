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
#include "slamwrapper.h"

using namespace std;
using namespace Eigen;


class EKFSLAMWrapper : public SLAMWrapper {
    Q_OBJECT

public:
    EKFSLAMWrapper(QObject *parent = 0);

    ~EKFSLAMWrapper();

protected:
    void run();

    void initializeDataAssociationTable();

    void drawCovarianceEllipseLines();

    vector<int> dataAssociationTable;
    MatrixXf P;

    EKFSLAM *algorithm;
};

#endif // __EKFSLAM_WRAPPER_H__
