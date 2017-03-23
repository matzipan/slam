//
// Created by matzipan on 07/02/17.
//

#include "Controller.h"

using namespace std;

Controller::Controller(WindowPlot *plot, QObject *parent) : QThread(parent)  {
    connect(this, SIGNAL(replot()), plot, SLOT(plot()));
    connect(this, SIGNAL(setCurrentIteration(int)), plot, SLOT(setCurrentIteration(int)));

    this->plot = plot;
    gatherer = new DataGatherer();

    context = new zmqpp::context();

    socket = new zmqpp::socket(*context, zmqpp::socket_type::pair);

    socket->bind("tcp://*:4242");
}

Controller::~Controller() {
    delete gatherer;
}

void Controller::run() {
    zmqpp::message message;

    while(continuePlotting) {
        if (socket->receive(message, true)) {
            string text;
            message>>text;

            if(text == "setLandmarks") {
                int32_t xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int32_t i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int32_t i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setLandmarks(xs, ys);
            } else if(text == "setWaypoints") {
                int32_t xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int32_t i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int32_t i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setWaypoints(xs, ys);
            } else if(text == "setParticles") {
                int32_t xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int32_t i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int32_t i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setParticles(xs, ys);
            } else if(text == "setFeatureParticles") {
                int32_t xSize, ySize;
                QVector<double> xs, ys;

                message>>xSize;
                for (int32_t i = 0; i < xSize; i++) {
                    double aux;
                    message>>aux;
                    xs.push_back(aux);
                }

                message>>ySize;
                for (int32_t i = 0; i < ySize; i++) {
                    double aux;
                    message>>aux;
                    ys.push_back(aux);
                }

                plot->setFeatureParticles(xs, ys);
            } else if(text == "setLaserLines") {
                int32_t rows, cols;

                message>>rows>>cols;
                Eigen::MatrixXf lnes(rows, cols);
                for (int32_t i = 0; i < rows; i++) {
                    for (int32_t j = 0; j < cols; j++) {
                        float aux;
                        message >> aux;
                        lnes(i, j) = aux;
                    }
                }

                plot->setLaserLines(lnes);
            } else if(text == "setCovEllipse") {
                int32_t rows, cols;
                int32_t idx;

                message>>rows>>cols;
                Eigen::MatrixXf lnes(rows, cols);
                for (int32_t i = 0; i < rows; i++) {
                    for (int32_t j = 0; j < cols; j++) {
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
                int32_t id;

                message>>s>>id;

                plot->setCarSize(s, id);
            } else if(text == "setCarTruePosition") {
                double x, y, t;

                message>>x>>y>>t;

                plot->setCarTruePosition(x,y,t);
                gatherer->setCarTruePosition(x,y,t);
            } else if(text == "setCarEstimatedPosition") {
                double x, y, t;

                message>>x>>y>>t;

                plot->setCarEstimatedPosition(x,y,t);
                gatherer->setCarEstimatedPosition(x,y,t);
            } else if(text == "setPlotRange") {
                double xmin, xmax, ymin, ymax;

                message>>xmin>>xmax>>ymin>>ymax;

                plot->setPlotRange(xmin, xmax, ymin, ymax);
            } else if(text == "clear") {
                plot->clear();
            } else if(text == "plot") {
                gatherer->nextTurn();
                emit replot();
            } else if(text == "setSimulationName") {
                std::string name;

                message>>name;

                gatherer->setSimulationName(name);
                //plot->setScreenshotFilename(name);
            } else if(text == "setCurrentIteration") {
                int32_t iteration;

                message>>iteration;

                emit setCurrentIteration(iteration);
            } else if(text == "covEllipseAdd") {
                int32_t n;
                message>>n;

                plot->covEllipseAdd(n);
            } else if(text == "endPlot") {
                gatherer->saveData();
                gatherer->cleanup();
            } else if(text == "loopTime") {
                uint32_t time;

                message>>time;

                gatherer->loopTime(time);
            }
        }
    }
}

void Controller::stop() {
    continuePlotting  = false;
}