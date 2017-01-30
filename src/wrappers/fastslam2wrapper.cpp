#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iterator>

#include <Eigen/Dense>

#include "src/core.h"
#include "src/plot.h"
#include "src/particle.h"
#include "src/algorithms/fastslam2.h"
#include "fastslam2wrapper.h"

using namespace std;
using namespace Eigen;

// global variable
extern Plot *gPlot;
extern Conf *gConf;


FastSLAM2Wrapper::FastSLAM2Wrapper(QObject *parent) : SLAMWrapper(parent) {
    algorithm = new FastSLAM2();
}

FastSLAM2Wrapper::~FastSLAM2Wrapper() { }


void FastSLAM2Wrapper::run() {
    printf("FastSLAM 2\n\n");

    if (gConf->SWITCH_PREDICT_NOISE) {
        printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");
    }
    if (gConf->SWITCH_SAMPLE_PROPOSAL == 0) {
        printf("Sampling from optimal proposal is usually ON for FastSLAM 2.0\n");
    }

    int pos_i = 0;
    double time_all;

    int m, n;

    QString msgAll;

    QVector<double> arrParticles_x, arrParticles_y;
    QVector<double> arrParticlesFea_x, arrParticlesFea_y;

    QVector<double> arrWaypoints_x, arrWaypoints_y;
    QVector<double> arrLandmarks_x, arrLandmarks_y;
    double x_min, x_max, y_min, y_max;

    double w_max;
    double x_mean, y_mean, t_mean;

    int draw_skip = 4;

    gConf->i("draw_skip", draw_skip);

    x_min = 1e30;
    x_max = -1e30;
    y_min = 1e30;
    y_max = -1e30;

    read_slam_input_file(map, &landmarks, &waypoints);

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

        gPlot->setWaypoints(arrWaypoints_x, arrWaypoints_y);
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

    gPlot->setLandmarks(arrLandmarks_x, arrLandmarks_y);

    gPlot->setCarSize(gConf->WHEELBASE, 0);
    gPlot->setCarSize(gConf->WHEELBASE, 1);
    gPlot->setPlotRange(x_min - (x_max - x_min) * 0.05, x_max + (x_max - x_min) * 0.05,
                         y_min - (y_max - y_min) * 0.05, y_max + (y_max - y_min) * 0.05);


    // Normally initialized configfile.h
    MatrixXf Q(2, 2), R(2, 2);

    Q << pow(gConf->sigmaV, 2), 0, 0, pow(gConf->sigmaG, 2);

    R << gConf->sigmaR * gConf->sigmaR, 0, 0, gConf->sigmaB * gConf->sigmaB;


    //vector of particles (their count will change)
    vector<Particle> particles(gConf->NPARTICLES);
    for (unsigned long i = 0; i < particles.size(); i++) {
        particles[i] = Particle();
    }

    //initialize particle weights as uniform
    float uniformw = 1.0 / (float) particles.size();
    for (int p = 0; p < particles.size(); p++) {
        particles[p].setW(uniformw);
    }

    VectorXf xtrue(3);
    xtrue.setZero();

    float dt = gConf->DT_CONTROLS; //change in time btw predicts
    float dtsum = 0; //change in time since last observation

    vector<int> ftag; //identifier for each landmark
    for (int i = 0; i < landmarks.cols(); i++) {
        ftag.push_back(i);
    }

    //data ssociation table
    VectorXf data_association_table(landmarks.cols());
    for (int s = 0; s < data_association_table.size(); s++) {
        data_association_table[s] = -1;
    }

    int iwp = 0; //index to first waypoint
    int nloop = gConf->NUMBER_LOOPS;
    float V = gConf->V; // default velocity
    float G = 0; //initial steer angle
    MatrixXf plines; //will later change to list of points

    if (gConf->SWITCH_SEED_RANDOM != 0) {
        srand(gConf->SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (gConf->SWITCH_INFLATE_NOISE == 1) {
        Qe = 2 * Q;
        Re = 2 * R;
    }


    pos_i = 0;
    time_all = 0.0;

    // initial position
    gPlot->addPos(xtrue(0), xtrue(1));
    gPlot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 0);

    gPlot->addPosEst(xtrue(0), xtrue(1));
    gPlot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 1);

    emit replot();

    float *VnGn = new float[2];
    float Vn, Gn;
    float V_ori = V;
    int cmd;
    vector<VectorXf> z; // Range and bearings of visible landmarks

    // Main loop
    while (isAlive) {
        if (runMode == SLAM_WAYPOINT) {
            if (iwp == -1) {
                break;
            }

            compute_steering(xtrue, waypoints, iwp, gConf->AT_WAYPOINT, G, gConf->RATEG, gConf->MAXG, dt);

            if (iwp == -1 && nloop > 1) {
                iwp = 0;
                nloop--;
            }
        }
        if (runMode == SLAM_INTERACTIVE) {
            getCommand(&cmd);

            // No commands then continue
            if (cmd == -1) {
                continue;
            }

            switch (cmd) {
                case 1:
                    // Forward
                    V = V_ori;
                    G = 0.0;
                    break;
                case 2:
                    // Backward
                    V = -V_ori;
                    G = 0.0;
                    break;
                case 3:
                    // Turn left
                    V = V_ori;
                    G = 30.0 * M_PI / 180.0;
                    break;
                case 4:
                    // Turn right
                    V = V_ori;
                    G = -30.0 * M_PI / 180.0;
                    break;
                default:
                    V = V_ori;
                    G = 0.0;
            }
        }

        // Predict current position and angle
        predict_true(xtrue, V, G, gConf->WHEELBASE, dt);

        // Add process noise
        add_control_noise(V, G, Q, gConf->SWITCH_CONTROL_NOISE, VnGn);
        Vn = VnGn[0];
        Gn = VnGn[1];

        // @TODO what happens when the if statement is false and the predict happens but not the observation
        // @TODO why does fastslam1 use Q and fastslam 2 use Qe
        algorithm->predict(particles, xtrue, Vn, Gn, Qe, gConf->WHEELBASE, dt, gConf->SWITCH_PREDICT_NOISE == 1, gConf->SWITCH_HEADING_KNOWN == 1);

        dtsum += dt;
        bool observe = false;

        if (dtsum >= gConf->DT_OBSERVE) {
            observe = true;
            dtsum = 0;

            vector<int> ftag_visible = vector<int>(ftag); // Modify the copy, not the ftag

            // Compute true data, then add noise
            // z is the range and bearing of the observed landmark
            z = get_observations(xtrue, landmarks, ftag_visible, gConf->MAX_RANGE);
            add_observation_noise(z, R, gConf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xtrue);

            // Compute (known) data associations
            unsigned long Nf = particles[0].xf().size();
            vector<int> idf;
            vector<VectorXf> zf;
            vector<VectorXf> zn;

            data_associate_known(z, ftag_visible, data_association_table, Nf, zf, idf, zn);

            // @TODO why does fastslam1 use R and fastslam 2 use Re
            algorithm->update(particles, zf, zn, idf, z, ftag_visible, data_association_table, Re, gConf->NEFFECTIVE, gConf->SWITCH_RESAMPLE == 1);
        }


        // Update status bar
        time_all = time_all + dt;
        pos_i++;

        // Accelate drawing speed
        if (pos_i % draw_skip != 0) {
            continue;
        }

        msgAll.sprintf("[%6d] %7.3f", pos_i, time_all);
        emit showMessage(msgAll);

        // get mean x, y
        x_mean = 0;
        y_mean = 0;
        t_mean = 0;
        w_max = -1e30;
        for (int i = 0; i < particles.size(); i++) {
            if (particles[i].w() > w_max) {
                w_max = particles[i].w();
                t_mean = particles[i].xv()(2);
            }
            x_mean += particles[i].xv()(0);
            y_mean += particles[i].xv()(1);
            //t_mean += pi_to_pi(particles[i].xv()(2));
        }

        x_mean = x_mean / particles.size();
        y_mean = y_mean / particles.size();
        //t_mean = t_mean / NPARTICLES;
        //printf("   x, y, t = %f %f %f\n", x_mean, y_mean, t_mean);

        // Draw particles
        arrParticles_x.clear();
        arrParticles_y.clear();
        for (int i = 0; i < particles.size(); i++) {
            arrParticles_x.push_back(particles[i].xv()(0));
            arrParticles_y.push_back(particles[i].xv()(1));
        }
        gPlot->setParticles(arrParticles_x, arrParticles_y);

        // Draw feature particles
        arrParticlesFea_x.clear();
        arrParticlesFea_y.clear();
        for (int i = 0; i < particles.size(); i++) {
            for (unsigned long j = 0; j < particles[i].xf().size(); j++) {
                arrParticlesFea_x.push_back(particles[i].xf()[j](0));
                arrParticlesFea_y.push_back(particles[i].xf()[j](1));
            }
        }
        gPlot->setParticlesFea(arrParticlesFea_x, arrParticlesFea_y);

        // Add new position
        if (pos_i % 4 == 0) {
            gPlot->addPos(xtrue(0), xtrue(1));
            gPlot->addPosEst(x_mean, y_mean);
        }

        // Draw current position
        gPlot->setCarPos(xtrue(0), xtrue(1), xtrue(2));
        gPlot->setCarPos(x_mean, y_mean, t_mean, 1);

        // Set laser lines
        gPlot->setLaserLines(plines);

        emit replot();

        msleep(10);
    }

    delete[] VnGn;
}
