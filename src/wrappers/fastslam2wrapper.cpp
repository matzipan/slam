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


FastSLAM2Wrapper::FastSLAM2Wrapper(QObject *parent) : ParticleSLAMWrapper(parent) {
    algorithm = new FastSLAM2();
}

FastSLAM2Wrapper::~FastSLAM2Wrapper() {
    wait();
}


void FastSLAM2Wrapper::run() {
    printf("FastSLAM 2\n\n");

    if (gConf->SWITCH_PREDICT_NOISE) {
        printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");
    }
    if (gConf->SWITCH_SAMPLE_PROPOSAL == 0) {
        printf("Sampling from optimal proposal is usually ON for FastSLAM 2.0\n");
    }

    configurePlot();
    initializeParticles();

    QVector<double> arrParticles_x, arrParticles_y;
    QVector<double> arrParticlesFea_x, arrParticlesFea_y;

    double w_max;
    double x_mean, y_mean, t_max;




    vector<int> ftag; //identifier for each landmark
    for (int i = 0; i < landmarks.cols(); i++) {
        ftag.push_back(i);
    }

    //data ssociation table
    VectorXf data_association_table(landmarks.cols());
    for (int s = 0; s < data_association_table.size(); s++) {
        data_association_table[s] = -1;
    }

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

    vector<VectorXf> z; // Range and bearings of visible landmarks

    // Main loop
    while (isAlive) {
        int controlStatus = control();

        if(controlStatus == -1) {
            break;
        }

        if(controlStatus == 0) {
            continue;
        }


        // @TODO what happens when the if statement is false and the predict happens but not the observation
        // @TODO why does fastslam1 use Q and fastslam 2 use Qe
        algorithm->predict(particles, xTrue, Vn, Gn, Qe, gConf->WHEELBASE, dt, gConf->SWITCH_PREDICT_NOISE == 1, gConf->SWITCH_HEADING_KNOWN == 1);

        dtSum += dt;
        bool observe = false;

        if (dtSum >= gConf->DT_OBSERVE) {
            observe = true;
            dtSum = 0;

            vector<int> ftag_visible = vector<int>(ftag); // Modify the copy, not the ftag

            // Compute true data, then add noise
            // z is the range and bearing of the observed landmark
            z = get_observations(xTrue, landmarks, ftag_visible, gConf->MAX_RANGE);
            add_observation_noise(z, R, gConf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xTrue);

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
        currentIteration++;

        // Accelate drawing speed
        if (currentIteration % drawSkip != 0) {
            continue;
        }

        plotMessage.sprintf("[%6d]", currentIteration);
        emit showMessage(plotMessage);

        // get mean x, y
        x_mean = 0;
        y_mean = 0;
        // The angle corresponding to the particle with the highest weight
        t_max = 0;
        w_max = -1e30;
        for (int i = 0; i < particles.size(); i++) {
            if (particles[i].w() > w_max) {
                w_max = particles[i].w();
                t_max = particles[i].xv()(2);
            }
            x_mean += particles[i].xv()(0);
            y_mean += particles[i].xv()(1);
        }

        x_mean = x_mean / particles.size();
        y_mean = y_mean / particles.size();

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
        gPlot->addTruePosition(xTrue(0), xTrue(1));
        gPlot->addEstimatedPosition(x_mean, y_mean);

        // Draw current position
        gPlot->setCarTruePosition(xTrue(0), xTrue(1), xTrue(2));
        gPlot->setCarEstimatedPosition(x_mean, y_mean, t_max);

        // Set laser lines
        gPlot->setLaserLines(plines);

        emit replot();
    }

    delete[] VnGn;
}
