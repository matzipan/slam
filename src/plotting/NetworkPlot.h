//
// Created by matzipan on 06/02/17.
//

#ifndef SLAM_GUI_NETWORKPLOT_H
#define SLAM_GUI_NETWORKPLOT_H


#include <QtCore>
#include <QtGui>
#include <QtCore>
#include <Eigen/Dense>
#include <libs/qcustomplot/qcustomplot.h>
#include <zmqpp/zmqpp.hpp>


class NetworkPlot : public QObject {
Q_OBJECT

public:
    NetworkPlot();

    ~NetworkPlot();

    void setLandmarks(QVector<double> &xs, QVector<double> &ys);
    void setWaypoints(QVector<double> &x, QVector<double> &ys);
    void setParticles(QVector<double> &x, QVector<double> &ys);
    void setFeatureParticles(QVector<double> &x, QVector<double> &ys);
    void setLaserLines(Eigen::MatrixXf &lnes);
    void setCovEllipse(Eigen::MatrixXf &lnes, int idx);
    void addTruePosition(double x, double y);
    void addEstimatedPosition(double x, double y);
    void setCarSize(double s, int id = 0);
    void setCarTruePosition(double x, double y, double t);
    void setCarEstimatedPosition(double x, double y, double t);
    void setPlotRange(double xmin, double xmax, double ymin, double ymax);
    void clear(void);
    void setScreenshotFilename(std::string filename);


public slots:

    void plot(void);
    void setCurrentIteration(int iteration);

    void canvasMousePressEvent(QMouseEvent *event);
    void covEllipseAdd(int n);

    signals:

    void commandSend(int cmd);
    void addCovEllipse(int n);


protected:
    zmqpp::socket *socket;
    zmqpp::context *context;

    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);

    void sendXYArrays(zmqpp::message &message, QVector<double> &xs, QVector<double> &ys);
};


#endif //SLAM_GUI_NETWORKPLOT_H
