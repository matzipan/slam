#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <QApplication>
#include "src/plotting/WindowPlot.h"
#include "src/plotting/WindowPlot.h"

#include "wrappers/slamwrapper.h"
#include "core.h"

#include "wrappers/fastslam1wrapper.h"
#include "wrappers/fastslam2wrapper.h"
#include "wrappers/ekfslamwrapper.h"

using namespace std;

void printUsage(char **argv) {
    printf("%s\n", argv[0]);
    printf("    -m                  [s] input map file name\n");
    printf("    -mode               [s] runing mode\n");
    printf("        waypoints   : following given waypoints\n");
    printf("        interactive : use keyboard to control movement\n");
    printf("    -method             [s] SLAM method\n");
    printf("        EKF1        : EKF SLAM 1\n");
    printf("        FAST1       : FastSLAM 1\n");
    printf("        FAST2       : FastSLAM 2\n");
    printf("    -h  (print usage)\n");
    printf("\n");
}

void loadConfiguration(int argc, char *argv[], Conf *conf) {
    // Parse input arguments
    string mapFilename = "data/example_webmap.mat";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && strlen(argv[i]) == 2) {
            mapFilename = argv[i + 1];
        }
        if (strcmp(argv[i], "-h") == 0) {
            printUsage(argv);
            exit(0);
        }
    }

    printf("map: %s\n", mapFilename.c_str());
    StringArray arrFN = path_splitext(mapFilename);
    string conf_fname = arrFN[0] + ".ini";

    conf->load(conf_fname);                  // First load conf file
    conf->set_args(argc, argv);              // Set input arguments
    conf->parse();                           // Parse input arguments

    // Save input arguments
    if (arrFN[0].size() > 0) {
        save_arguments(argc, argv, arrFN[0]);
    } else {
        save_arguments(argc, argv, "fastslam");
    }

    // Print parameters
    conf->print();
}

void configurePlot(NetworkPlot *plot, Conf *conf) {
    plot->setScreenshotFilename(conf->s("fn_screenshot"));
    plot->plot();
}

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);
    Conf conf;

    loadConfiguration(argc, argv, &conf);

    NetworkPlot plot;
    plot.setScreenshotFilename(conf.s("fn_screenshot"));

    SLAMWrapper *slamThread;

    string slamMethod = conf.s("method");
    if (slamMethod == "FAST1") {  slamThread = new FastSLAM1Wrapper(&conf, &plot); }
    else if (slamMethod == "FAST2") {  slamThread = new FastSLAM2Wrapper(&conf, &plot); }
    else {  slamThread = new EKFSLAMWrapper(&conf, &plot); }

    // Begin SLAM simulation
    slamThread->start();

    int returnValue = application.exec();

    slamThread->stop();

    delete slamThread;

    return returnValue;
}
