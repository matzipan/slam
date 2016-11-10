#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/plot.h"
#include "src/algorithms/ekfslam.h"
#include "ekfslam_wrapper.h"

using namespace std;
using namespace Eigen;

// global variable
extern Plot *g_plot;
extern Conf *g_conf;


EKFSLAM_Wrapper::EKFSLAM_Wrapper(QObject *parent) : Wrapper_Thread(parent) {
    algorithm = new EKFSLAM();
}

EKFSLAM_Wrapper::~EKFSLAM_Wrapper() {
    wait();
}


void EKFSLAM_Wrapper::run() {
    printf("EKFSLAM\n\n");

    MatrixXf landmarks; //landmark positions
    MatrixXf waypoints; //way points

    int pos_i = 0;
    double time_all;

    int m, n;

    QString msgAll;

    QVector<double> arrWaypoints_x, arrWaypoints_y;
    QVector<double> arrLandmarks_x, arrLandmarks_y;
    double x_min, x_max, y_min, y_max;

    int draw_skip = 4;

    g_conf->i("draw_skip", draw_skip);

    x_min = 1e30;
    x_max = -1e30;
    y_min = 1e30;
    y_max = -1e30;

    read_slam_input_file(fnMap, &landmarks, &waypoints);

    // draw waypoints
    if (runMode == SLAM_WAYPOINT) {
        m = waypoints.rows();
        n = waypoints.cols();

        for (int i = 0; i < n; i++) {
            arrWaypoints_x.push_back(waypoints(0, i));
            arrWaypoints_y.push_back(waypoints(1, i));

            if (waypoints(0, i) > x_max) { x_max = waypoints(0, i); }
            if (waypoints(0, i) < x_min) { x_min = waypoints(0, i); }
            if (waypoints(1, i) > y_max) { y_max = waypoints(1, i); }
            if (waypoints(1, i) < y_min) { y_min = waypoints(1, i); }
        }

        g_plot->setWaypoints(arrWaypoints_x, arrWaypoints_y);
    }

    // draw landmarks
    m = landmarks.rows();
    n = landmarks.cols();
    for (int i = 0; i < n; i++) {
        arrLandmarks_x.push_back(landmarks(0, i));
        arrLandmarks_y.push_back(landmarks(1, i));

        if (landmarks(0, i) > x_max) { x_max = landmarks(0, i); }
        if (landmarks(0, i) < x_min) { x_min = landmarks(0, i); }
        if (landmarks(1, i) > y_max) { y_max = landmarks(1, i); }
        if (landmarks(1, i) < y_min) { y_min = landmarks(1, i); }
    }

    g_plot->setLandmarks(arrLandmarks_x, arrLandmarks_y);

    g_plot->setCarSize(g_conf->WHEELBASE, 0);
    g_plot->setCarSize(g_conf->WHEELBASE, 1);
    g_plot->setPlotRange(x_min - (x_max - x_min) * 0.05, x_max + (x_max - x_min) * 0.05,
                         y_min - (y_max - y_min) * 0.05, y_max + (y_max - y_min) * 0.05);

    //normally initialized configfile.h
    MatrixXf Q(2, 2), R(2, 2);
    float sigma_phi;           // radians, heading uncertainty

    Q << pow(g_conf->sigmaV, 2), 0, 0, pow(g_conf->sigmaG, 2);

    R << g_conf->sigmaR * g_conf->sigmaR, 0, 0, g_conf->sigmaB * g_conf->sigmaB;

    sigma_phi = g_conf->sigmaT;

    VectorXf xtrue(3);
    VectorXf x(3, 1);
    MatrixXf P(3, 3);

    xtrue.setZero(3);
    x.setZero(3);
    P.setZero(3, 3);

    float dt = g_conf->DT_CONTROLS; //change in time btw predicts
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

    int iwp = 0;    //index to first waypoint
    int nloop = g_conf->NUMBER_LOOPS;
    float V = g_conf->V;  // default velocity
    float G = 0;      //initial steer angle
    MatrixXf plines;     //will later change to list of points
    MatrixXf covLines;   // covariance ellipse lines

    if (g_conf->SWITCH_SEED_RANDOM != 0) {
        srand(g_conf->SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (g_conf->SWITCH_INFLATE_NOISE == 1) {
        Qe = 2 * Q;
        Re = 8 * R;
    }

    vector<int> idf;

    vector<VectorXf> z;              //range and bearings of visible landmarks
    vector<VectorXf> zf;
    vector<VectorXf> zn;

    pos_i = 0;
    time_all = 0.0;

    // initial position
    g_plot->addPos(xtrue(0), xtrue(1));
    g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 0);

    g_plot->addPosEst(xtrue(0), xtrue(1));
    g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 1);

    emit replot();

    float *VnGn = new float[2];
    float Vn, Gn;
    float V_ori = V;
    int cmd;

    //Main loop
    while (isAlive) {
        if (runMode == SLAM_WAYPOINT) {
            if (iwp == -1) {
                break;
            }

            compute_steering(xtrue, waypoints, iwp, g_conf->AT_WAYPOINT, G, g_conf->RATEG, g_conf->MAXG, dt);

            if (iwp == -1 && nloop > 1) {
                iwp = 0;
                nloop--;
            }
        }
        if (runMode == SLAM_INTERACTIVE) {
            getCommand(&cmd);

            // no commands then continue
            if (cmd == -1) {
                continue;
            }

            switch (cmd) {
                case 1:
                    // forward
                    V = V_ori;
                    G = 0.0;
                    break;
                case 2:
                    // backward
                    V = -V_ori;
                    G = 0.0;
                    break;
                case 3:
                    // turn left
                    V = V_ori;
                    G = 30.0 * M_PI / 180.0;
                    break;
                case 4:
                    // turn right
                    V = V_ori;
                    G = -30.0 * M_PI / 180.0;
                    break;
                default:
                    V = V_ori;
                    G = 0.0;
            }
        }

        // get true position
        predict_true(xtrue, V, G, g_conf->WHEELBASE, dt);

        // add process noise
        add_control_noise(V, G, Q, g_conf->SWITCH_CONTROL_NOISE, VnGn);
        Vn = VnGn[0];
        Gn = VnGn[1];

        dtsum += dt;
        bool observe = false;

        vector<int> ftag_visible = vector<int>(ftag); //modify the copy, not the ftag

        if (dtsum >= g_conf->DT_OBSERVE) {
            observe = true;
            dtsum = 0;

            // z is the range and bearing of the observed landmark
            z = get_observations(xtrue, landmarks, ftag_visible, g_conf->MAX_RANGE);
            add_observation_noise(z, R, g_conf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xtrue);
        }

        algorithm->sim(landmarks, waypoints, x, P, Vn, Gn, Qe,
                       g_conf->WHEELBASE, dt, xtrue(2) + g_conf->sigmaT * unifRand(), g_conf->SWITCH_HEADING_KNOWN,
                       sigma_phi, ftag,
                       z, Re, g_conf->GATE_REJECT, g_conf->GATE_AUGMENT, g_conf->SWITCH_ASSOCIATION_KNOWN, observe, zf,
                       idf, zn,
                       data_association_table, g_conf->SWITCH_BATCH_UPDATE == 1, R);

        // update status bar
        time_all = time_all + dt;
        pos_i++;

        // accelate drawing speed
        if (pos_i % draw_skip != 0) {
            continue;
        }

        msgAll.sprintf("[%6d] %7.3f", pos_i, time_all);
        emit showMessage(msgAll);

        // add new position
        if (pos_i % 4 == 0) {
            g_plot->addPos(xtrue(0), xtrue(1));
            g_plot->addPosEst(x(0), x(1));
        }

        // draw current position
        g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2));
        g_plot->setCarPos(x(0), x(1), x(2), 1);

        // set laser lines
        g_plot->setLaserLines(plines);

        // set covariance ellipse lines
        MatrixXf x_(2, 1);
        MatrixXf P_ = P.block(0, 0, 2, 2);
        x_(0) = x(0);
        x_(1) = x(1);

        make_covariance_ellipse(x_, P_, covLines);
        g_plot->setCovEllipse(covLines, 0);

        int j = (x.size() - 3) / 2;
        for (int i = 0; i < j; i++) {
            x_(0) = x(3 + i * 2);
            x_(1) = x(3 + i * 2 + 1);
            P_ = P.block(3 + i * 2, 3 + i * 2, 2, 2);

            make_covariance_ellipse(x_, P_, covLines);
            g_plot->setCovEllipse(covLines, i + 1);
        }

        emit replot();

        msleep(10);
    }

    if (VnGn) {
        delete[] VnGn;
    }
}

