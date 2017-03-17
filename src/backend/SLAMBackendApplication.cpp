//
// Created by matzipan on 13/02/17.
//

#include <wrappers/slamwrapper.h>
#include <wrappers/fastslam1wrapper.h>
#include <wrappers/fastslam2wrapper.h>
#include <wrappers/ekfslamwrapper.h>
#include "SLAMBackendApplication.h"

#ifdef JACOBIAN_ACCELERATOR
    /// Pointer to zynq handler to link against using extern
    AcceleratorHandler* acceleratorHandler;
#endif


SLAMBackendApplication::SLAMBackendApplication(int &argc, char **argv)  {
    loadConfiguration(argc, argv);

    plot.setScreenshotFilename(conf.s("fn_screenshot"));

#ifdef JACOBIAN_ACCELERATOR
    acceleratorHandler = new AcceleratorHandler();
#endif

    string slamMethod = conf.s("method");
    if (slamMethod == "FASTSLAM1") {  slamThread = new FastSLAM1Wrapper(&conf, &plot); }
    else if (slamMethod == "FASTSLAM2") {  slamThread = new FastSLAM2Wrapper(&conf, &plot); }
    else {  slamThread = new EKFSLAMWrapper(&conf, &plot); }

}

SLAMBackendApplication::~SLAMBackendApplication () {
    delete slamThread;
#ifdef JACOBIAN_ACCELERATOR
    delete acceleratorHandler;
#endif
}

void SLAMBackendApplication::run() {
    slamThread->run();
}

void SLAMBackendApplication::printUsage(char **argv) {
    printf("%s\n", argv[0]);
    printf("    -m                  [s] input map file name\n");
    printf("    -mode               [s] runing mode\n");
    printf("        waypoints   : following given waypoints\n");
    printf("        interactive : use keyboard to control movement\n");
    printf("    -method             [s] SLAM method\n");
    printf("        EKF1        : EKF SLAM 1\n");
    printf("        FASTSLAM1       : FastSLAM 1\n");
    printf("        FASTSLAM2       : FastSLAM 2\n");
    printf("    -h  (print usage)\n");
    printf("\n");
}

void SLAMBackendApplication::loadConfiguration(int &argc, char **argv) {
    // Parse input arguments
    string mapFilename = "example_webmap.mat";

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

    // Print parameters
    conf.print();
}
