//
// Created by matzipan on 07/02/17.
//

#ifndef SLAM_GUI_PLOTCONTROLLER_H
#define SLAM_GUI_PLOTCONTROLLER_H


#include "WindowPlot.h"
#include <zmqpp/zmqpp.hpp>


class PlotController : public QThread {
Q_OBJECT

public:
    PlotController (WindowPlot *plot, QObject *parent = 0);
    ~PlotController ();

signals:
    void replot();
    void showMessage(QString msg);

protected:
    WindowPlot *plot;
    zmqpp::socket *socket;
    zmqpp::context *context;

    void run();
};


#endif //SLAM_GUI_PLOTCONTROLLER_H
