//
// Created by matzipan on 13/02/17.
//

#ifndef SLAM_GUI_SLAMBACKENDAPPLICATION_H
#define SLAM_GUI_SLAMBACKENDAPPLICATION_H

#include "core.h"
#include "plotting/NetworkPlot.h"
#include "wrappers/slamwrapper.h"

#ifdef JACOBIAN_ACCELERATOR
    #include "AcceleratorHandler.h"
#endif

class SLAMBackendApplication {

public:
    SLAMBackendApplication(int &argc, char **argv);

    ~SLAMBackendApplication();

    void run();

protected:
    Conf conf;
    NetworkPlot plot;
    SLAMWrapper *slamThread;

    void loadConfiguration(int &argc, char **argv);

    void printUsage(char **argv);
};


#endif //SLAM_GUI_SLAMBACKENDAPPLICATION_H
