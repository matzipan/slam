//
// Created by matzipan on 13/02/17.
//

#include <src/wrappers/slamwrapper.h>
#include <src/wrappers/fastslam1wrapper.h>
#include <src/wrappers/fastslam2wrapper.h>
#include <src/wrappers/ekfslamwrapper.h>
#include "SLAMBackendApplication.h"

SLAMBackendApplication::SLAMBackendApplication(int &argc, char **argv) : QApplication(argc, argv) {
    loadConfiguration(argc, argv);

    plot.setScreenshotFilename(conf.s("fn_screenshot"));

    string slamMethod = conf.s("method");
    if (slamMethod == "FAST1") {  slamThread = new FastSLAM1Wrapper(&conf, &plot); }
    else if (slamMethod == "FAST2") {  slamThread = new FastSLAM2Wrapper(&conf, &plot); }
    else {  slamThread = new EKFSLAMWrapper(&conf, &plot); }

    connect(slamThread, SIGNAL(jobFinished()), this, SLOT(quit()));

    slamThread->start();
}

SLAMBackendApplication::~SLAMBackendApplication () {
    delete slamThread;
}

void SLAMBackendApplication::printUsage(char **argv) {
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

void SLAMBackendApplication::loadConfiguration(int &argc, char **argv) {
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

    conf.load(conf_fname);                  // First load conf file
    conf.set_args(argc, argv);              // Set input arguments
    conf.parse();                           // Parse input arguments

    // Save input arguments
    if (arrFN[0].size() > 0) {
        save_arguments(argc, argv, arrFN[0]);
    } else {
        save_arguments(argc, argv, "fastslam");
    }

    // Print parameters
    conf.print();
}
