#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <QApplication>


#include "plot.h"
#include "src/wrappers/slamwrapper.h"
#include "core.h"

#include "src/wrappers/fastslam1wrapper.h"
#include "src/wrappers/fastslam2wrapper.h"
#include "src/wrappers/ekfslamwrapper.h"

using namespace std;


////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////
Plot *gPlot;
Conf *gConf;


////////////////////////////////////////////////////////////////////////////////
/// Print usage help
////////////////////////////////////////////////////////////////////////////////
void print_usage(char *argv[]) {
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


////////////////////////////////////////////////////////////////////////////////
/// Main function
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
    int i;
    int returnValue = 0;
    SLAMWrapper *slamThread;

    string mode, method;
    string fn_screenshot;

    int ww, wh;

    int slamMethod = 1;
    SLAMWrapper::RunMode slamRunMode = SLAMWrapper::SLAM_WAYPOINT;
    string mapFilename = "../data/example_webmap.mat";
    string conf_fname;

    Conf conf;
    StringArray arrFN;

    gConf = &conf;

    dbg_stacktrace_setup();

    // parse input arguments
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && strlen(argv[i]) == 2) {
            mapFilename = argv[i + 1];
        }
        if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv);
            return 0;
        }
    }

    printf("map: %s\n", mapFilename.c_str());
    arrFN = path_splitext(mapFilename);
    conf_fname = arrFN[0] + ".ini";

    conf.load(conf_fname);                  // First load conf file
    conf.set_args(argc, argv);              // Set input arguments
    conf.parse();                           // Parse input arguments

    // Save input arguments
    if (arrFN[0].size() > 0) {
        save_arguments(argc, argv, arrFN[0]);
    } else {
        save_arguments(argc, argv, "fastslam");
    }

    // Get mode
    mode = "waypoints";
    conf.s("mode", mode);
    if (mode == "waypoints") slamRunMode = SLAMWrapper::SLAM_WAYPOINT;
    if (mode == "interactive") slamRunMode = SLAMWrapper::SLAM_INTERACTIVE;

    // Get SLAM method
    method = "EKF1";
    conf.s("method", method);
    if (method == "FAST1") slamMethod = 1;
    if (method == "FAST2") slamMethod = 2;
    if (method == "EKF1") slamMethod = 3;

    // Get screenshot filename
    fn_screenshot = "";
    conf.s("fn_screenshot", fn_screenshot);

    // get window size
    ww = 950;
    wh = 768;
    conf.i("ww", ww);
    conf.i("wh", wh);

    // Print parameters
    conf.print();

    QApplication application(argc, argv);

    // Configure plot window
    Plot w;
    gPlot = &w;
    w.show();
    w.setGeometry(10, 10, ww, wh);
    w.setScreenShot_fname(fn_screenshot);
    w.plot();

    // create SLAM thread
    switch (slamMethod) {
        case 1:
            slamThread = new FastSLAM1Wrapper;
            break;
        case 2:
            slamThread = new FastSLAM2Wrapper;
            break;
        case 3:
            slamThread = new EKFSLAMWrapper;
            break;
        default:
            return -1;
    }

    // Set run mode
    slamThread->setRunMode(slamRunMode);
    // Set map
    slamThread->loadMap(mapFilename);

    // Begin SLAM simulation
    slamThread->start();

    // Begin GUI loop
    returnValue = application.exec();

    // Stop SLAM thread
    slamThread->stop();

    delete slamThread;

    return returnValue;
}
