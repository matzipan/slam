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

#include "src/backend/core.h"
#include "src/backend/algorithms/ekfslam.h"
#include "slamwrapper.h"

using namespace std;
using namespace Eigen;


class EKFSLAMWrapper : public SLAMWrapper {

public:
    EKFSLAMWrapper(Conf *conf, NetworkPlot *plot);

    ~EKFSLAMWrapper();

    void run();

protected:

    void initializeDataAssociationTable();

    void drawCovarianceEllipseLines();

    vector<int> dataAssociationTable;

    /// A posteriori error covariance matrix (a measure of the estimated accuracy of the state estimate).
    MatrixXf P;

    EKFSLAM *algorithm;
};

#endif // __EKFSLAM_WRAPPER_H__
