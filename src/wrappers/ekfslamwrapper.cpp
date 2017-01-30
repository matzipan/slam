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

// global variable
extern Plot *gPlot;
extern Conf *gConf;


EKFSLAMWrapper::EKFSLAMWrapper(QObject *parent) : SLAMWrapper(parent) {
    algorithm = new EKFSLAM();
}

EKFSLAMWrapper::~EKFSLAMWrapper() { }


void EKFSLAMWrapper::run() {
    printf("EKFSLAM\n\n");

    double time_all;

    QString msgAll;

    int draw_skip = 4;

    gConf->i("draw_skip", draw_skip);

    read_slam_input_file(map, &landmarks, &waypoints);

    configurePlot();

    MatrixXf P(3, 3);

    P.setZero(3, 3);

    float dtsum = 0; //change in time since last observation

    vector<int> ftag; //identifier for each landmark
    for (int i = 0; i < landmarks.cols(); i++) {
        ftag.push_back(i);
    }

    //data ssociation table
    vector<int> data_association_table;
    for (int i = 0; i < landmarks.cols(); i++) {
        data_association_table.push_back(-1);
    }

    MatrixXf plines; //will later change to list of points
    MatrixXf covLines;   // covariance ellipse lines

    if (gConf->SWITCH_SEED_RANDOM != 0) {
        srand(gConf->SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (gConf->SWITCH_INFLATE_NOISE == 1) {
        Qe = 2 * Q;
        Re = 8 * R;
    }

    vector<int> idf;

    vector<VectorXf> z; //range and bearings of visible landmarks
    vector<VectorXf> zf;
    vector<VectorXf> zn;

    time_all = 0.0;

    // Main loop
    while (isAlive) {
        int controlStatus = control();

        if(controlStatus == -1) {
            break;
        }

        if(controlStatus == 0) {
            continue;
        }

        dtsum += dt;
        bool observe = false;

        if (dtsum >= gConf->DT_OBSERVE) {
            observe = true;
            dtsum = 0;

            //Compute true data, then add noise
            vector<int> ftag_visible = vector<int>(ftag); //modify the copy, not the ftag

            //z is the range and bearing of the observed landmark
            z = get_observations(xTrue, landmarks, ftag_visible, gConf->MAX_RANGE);
            add_observation_noise(z, R, gConf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xTrue);
        }

        // @TODO what happens if this is moved inside the if(OBSERVE) branch?
        algorithm->sim(landmarks, waypoints, x, P, Vn, Gn, Qe,
                       gConf->WHEELBASE, dt, xTrue(2) + gConf->sigmaT * unifRand(), gConf->SWITCH_HEADING_KNOWN,
                       sigmaPhi, ftag,
                       z, Re, gConf->GATE_REJECT, gConf->GATE_AUGMENT, gConf->SWITCH_ASSOCIATION_KNOWN, observe, zf,
                       idf, zn,
                       data_association_table, gConf->SWITCH_BATCH_UPDATE == 1, R);

        // update status bar
        time_all = time_all + dt;
        currentIteration++;

        // accelate drawing speed
        if (currentIteration % draw_skip != 0) {
            continue;
        }

        msgAll.sprintf("[%6d] %7.3f", currentIteration, time_all);
        emit showMessage(msgAll);

        // add new position
        if (currentIteration % 4 == 0) {
            gPlot->addPos(xTrue(0), xTrue(1));
            gPlot->addPosEst(x(0), x(1));
        }

        // draw current position
        gPlot->setCarPos(xTrue(0), xTrue(1), xTrue(2));
        gPlot->setCarPos(x(0), x(1), x(2), 1);

        // set laser lines
        gPlot->setLaserLines(plines);

        // set covariance ellipse lines
        MatrixXf x_(2, 1);
        MatrixXf P_ = P.block(0, 0, 2, 2);
        x_(0) = x(0);
        x_(1) = x(1);

        make_covariance_ellipse(x_, P_, covLines);
        gPlot->setCovEllipse(covLines, 0);

        int j = (x.size() - 3) / 2;
        for (int i = 0; i < j; i++) {
            x_(0) = x(3 + i * 2);
            x_(1) = x(3 + i * 2 + 1);
            P_ = P.block(3 + i * 2, 3 + i * 2, 2, 2);

            make_covariance_ellipse(x_, P_, covLines);
            gPlot->setCovEllipse(covLines, i + 1);
        }

        emit replot();

        msleep(10);
    }

    delete[] VnGn;
}

