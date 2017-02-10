#ifndef SLAMPLOT_H
#define SLAMPLOT_H

#include <QtCore>
#include <QtGui>
#include <QMainWindow>
#include <QTimer>

#include <Eigen/Dense>

#include "libs/qcustomplot/qcustomplot.h"


class WindowPlot : public QMainWindow {
Q_OBJECT

public:
    explicit WindowPlot(QWidget *parent = 0);

    ~WindowPlot();

    void setupCanvas(void);
    void setupInitData(void);
    void setLandmarks(QVector<double> &arrX, QVector<double> &arrY);
    void setWaypoints(QVector<double> &arrX, QVector<double> &arrY);
    void setParticles(QVector<double> &arrX, QVector<double> &arrY);
    void setFeatureParticles(QVector<double> &arrX, QVector<double> &arr);
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

    void plot();
    void setCurrentIteration(int iteration);

    // @TODO These functions can be used to send messages from WindowPlot to NetworkPlot when an event happens
    void canvasMousePressEvent(QMouseEvent *event);
    void canvasMouseMoveEvent(QMouseEvent *event);

    void plotBegin(void);
    void plotEnd(void);

    void covEllipseAdd(int n);

signals:

    // @TODO These signals can be used to send messages from WindowPlot to NetworkPlot when an event happens
    void commandSend(int cmd);
    void addCovEllipse(int n);


protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);
    void setCarPos(double x, double y, double t, int id = 0);

private:
    QCustomPlot *customPlot;

    QVector<double> arrWaypoint_x, arrWaypoint_y;
    QVector<double> arrPos_x, arrPos_y;
    QVector<double> arrEstPos_x, arrEstPos_y;
    QVector<double> arrCurrPos_x, arrCurrPos_y;
    QVector<double> arrParticles_x, arrParticles_y;
    QVector<double> arrLandmarks_x, arrLandmarks_y;
    QVector<double> arrParticlesFea_x, arrParticlesFea_y;

    double parmCarModel[4];        // 0 - pos xEstimated
    // 1 - pos y
    // 2 - theta
    // 3 - size
    double parmCarEst[4];          // 0 - pos xEstimated
    // 1 - pos y
    // 2 - theta
    // 3 - size

    QCPGraph *plotParticles;
    QCPGraph *plotFeatureParticles;
    QCPGraph *plotLandmarks;
    QCPCurve *curvWayPoint;
    QCPCurve *curvRealPos;
    QCPCurve *curvEstPos;
    QCPCurve *curvCar;
    QCPCurve *curvCarEst;

    QVector<QCPGraph *> arrLaserLines;                  // laser line graph
    QVector<QCPCurve *> arrCovLines;                    // cov ellipse graph

    QString msgString1, msgString2;

    std::string fnScreenShot_base;

    QMutex *muxData;

private:
    void drawCar(int idx = 0);
};

#endif // SLAMPLOT_H
