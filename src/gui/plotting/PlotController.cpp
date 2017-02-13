//
// Created by matzipan on 07/02/17.
//

#include "PlotController.h"
using namespace std;

PlotController::PlotController(WindowPlot *plot, QObject *parent) : QThread(parent)  {
    connect(this, SIGNAL(replot()), plot, SLOT(plot()));
    connect(this, SIGNAL(setCurrentIteration(int)), plot, SLOT(setCurrentIteration(int)));

    this->plot = plot;

    context = new zmqpp::context ();

    socket = new zmqpp::socket (*context, zmqpp::socket_type::pull);

    socket->bind("tcp://*:4242");
}

PlotController::~PlotController () {}

void PlotController::run () {
    zmqpp::message message;

    while(continuePlotting) {
        if (socket->receive(message, true)) {
            string text;
            message >> text;

            if(text == "setLandmarks") {
                int xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setLandmarks(xs, ys);
            } else if(text == "setWaypoints") {
                int xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setWaypoints(xs, ys);
            } else if(text == "setParticles") {
                int xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setParticles(xs, ys);
            } else if(text == "setFeatureParticles") {
                int xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setFeatureParticles(xs, ys);
            } else if(text == "setLaserLines") {
                long rows, cols;

                message>>rows>>cols;
                Eigen::MatrixXf lnes(rows, cols);
                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                        float aux;
                        message >> aux;
                        lnes(i, j) = aux;
                    }
                }

                plot->setLaserLines(lnes);
            } else if(text == "setCovEllipse") {
                long rows, cols;
                int idx;

                message>>rows>>cols;
                Eigen::MatrixXf lnes(rows, cols);
                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                        float aux;
                        message >> aux;
                        lnes(i, j) = aux;
                    }
                }

                message>>idx;

                plot->setCovEllipse(lnes, idx);
            } else if(text == "addTruePosition") {
                double x, y;

                message>>x>>y;

                plot->addTruePosition(x,y);
            } else if(text == "addEstimatedPosition") {
                double x, y;

                message>>x>>y;

                plot->addEstimatedPosition(x,y);
            } else if(text == "setCarSize") {
                double s;
                int id;

                message>>s>>id;

                plot->setCarSize(s, id);
            } else if(text == "setCarTruePosition") {
                double x, y, t;

                message>>x>>y>>t;

                plot->setCarTruePosition(x,y,t);
            } else if(text == "setCarEstimatedPosition") {
                double x, y, t;

                message>>x>>y>>t;

                plot->setCarEstimatedPosition(x,y,t);
            } else if(text == "setPlotRange") {
                double xmin, xmax, ymin, ymax;

                message>>xmin>>xmax>>ymin>>ymax;

                plot->setPlotRange(xmin, xmax, ymin, ymax);
            } else if(text == "clear") {
                plot->clear();
            } else if(text == "plot") {
                emit replot();
            } else if(text == "setScreenshotFilename") {
                std::string filename;

                message>>filename;

                plot->setScreenshotFilename(filename);
            } else if(text == "setCurrentIteration") {
                int iteration;

                message>>iteration;

                emit setCurrentIteration(iteration);
            } else if(text == "covEllipseAdd") {
                int n;
                message>>n;

                plot->covEllipseAdd(n);
            }
        }
    }
}

void PlotController::stop() {
    continuePlotting  = false;
}