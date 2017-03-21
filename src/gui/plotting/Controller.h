//
// Created by matzipan on 07/02/17.
//

#ifndef SLAM_GUI_PLOTCONTROLLER_H
#define SLAM_GUI_PLOTCONTROLLER_H


#include "WindowPlot.h"
#include "DataGatherer.h"
#include <zmqpp/zmqpp.hpp>


class Controller : public QThread {
Q_OBJECT

public:
    Controller (WindowPlot *plot, QObject *parent = 0);
    ~Controller ();

    void stop();

signals:
    void replot();
    void setCurrentIteration(int iteration);

protected:
    WindowPlot *plot;
    DataGatherer *gatherer;
    zmqpp::socket *socket;
    zmqpp::context *context;

    void run();

    bool continuePlotting = true;

};


#endif //SLAM_GUI_PLOTCONTROLLER_H
