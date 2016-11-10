#ifndef __FASTSLAM_1_H__
#define __FASTSLAM_1_H__

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
#include "wrapper_thread.h"

using namespace std;
using namespace Eigen;


class FastSLAM1_Wrapper : public Wrapper_Thread {
    Q_OBJECT

public:
    FastSLAM1_Wrapper(QObject *parent = 0);

    ~FastSLAM1_Wrapper();

    void predict(Particle &particle, float V, float G, MatrixXf &Q, float WB, float dt, int addrandom);

    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    vector<Particle> sim(MatrixXf &lm, MatrixXf &wp);


protected:
    void run();
};

#endif // __FASTSLAM_1_H__