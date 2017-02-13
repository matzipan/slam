//
// Created by matzipan on 13/02/17.
//

#ifndef SLAM_GUI_SLAMBACKENDAPPLICATION_H
#define SLAM_GUI_SLAMBACKENDAPPLICATION_H

#include "core.h"
#include "plotting/NetworkPlot.h"
#include "wrappers/slamwrapper.h"
#include <QtGui/QApplication>

class SLAMBackendApplication : public QApplication {
Q_OBJECT

public:
    SLAMBackendApplication(int &argc, char **argv);

    ~SLAMBackendApplication();

protected:
    Conf conf;
    NetworkPlot plot;
    SLAMWrapper *slamThread;

    void loadConfiguration(int &argc, char **argv);

    void printUsage(char **argv);

};


#endif //SLAM_GUI_SLAMBACKENDAPPLICATION_H
