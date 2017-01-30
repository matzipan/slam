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
// global variables
////////////////////////////////////////////////////////////////////////////////
Plot *gPlot;
Conf *gConf;


////////////////////////////////////////////////////////////////////////////////
/// print usage help
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
/// main function
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
    int i;
    int ret = 0;
    SLAMWrapper *slam_thread;

    string mode, method;
    string fn_screenshot;

    int ww, wh;

    int slam_method = 1;
    SLAMWrapper::RunMode slam_runmode = SLAMWrapper::SLAM_WAYPOINT;
    string map_fname = "../data/example_webmap.mat";
    string conf_fname;

    Conf conf;
    StringArray arrFN;

    gConf = &conf;

    dbg_stacktrace_setup();

    // parse input arguments
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && strlen(argv[i]) == 2) {
            map_fname = argv[i + 1];
        }
        if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv);
            return 0;
        }
    }

    printf("map: %s\n", map_fname.c_str());
    arrFN = path_splitext(map_fname);
    conf_fname = arrFN[0] + ".ini";

    conf.load(conf_fname);                  // first load conf file
    conf.set_args(argc, argv);              // set input arguments
    conf.parse();                           // parse input arguments

    // save input arguments
    if (arrFN[0].size() > 0) {
        save_arguments(argc, argv, arrFN[0]);
    } else {
        save_arguments(argc, argv, "fastslam");
    }

    // get mode
    mode = "waypoints";
    conf.s("mode", mode);
    if (mode == "waypoints") slam_runmode = SLAMWrapper::SLAM_WAYPOINT;
    if (mode == "interactive") slam_runmode = SLAMWrapper::SLAM_INTERACTIVE;

    // get slam method
    method = "EKF1";
    conf.s("method", method);
    if (method == "FAST1") slam_method = 1;
    if (method == "FAST2") slam_method = 2;
    if (method == "EKF1") slam_method = 3;

    // get screenshot filename
    fn_screenshot = "";
    conf.s("fn_screenshot", fn_screenshot);

    // get window size
    ww = 950;
    wh = 768;
    conf.i("ww", ww);
    conf.i("wh", wh);

    // print parameters
    conf.print();

    // Begin QT application
    QApplication a(argc, argv);

    // SLAM plot window
    Plot w;
    gPlot = &w;
    w.show();
    w.setGeometry(10, 10, ww, wh);
    w.setScreenShot_fname(fn_screenshot);
    w.plot();

    // create SLAM thread
    switch (slam_method) {
        case 1:
            slam_thread = new FastSLAM1Wrapper;
            break;
        case 2:
            slam_thread = new FastSLAM2Wrapper;
            break;
        case 3:
            slam_thread = new EKFSLAMWrapper;
            break;
        default:
            return -1;
    }

    // set run mode
    slam_thread->setRunMode(slam_runmode);
    // set map
    slam_thread->setMap(map_fname);

    // begin SLAM simulation
    slam_thread->start();

    // begin GUI loop
    ret = a.exec();

    // stop SLAM thread
    slam_thread->stop();

    delete slam_thread;

    return ret;
}
