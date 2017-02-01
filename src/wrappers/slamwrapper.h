#ifndef SLAM_THREAD_H
#define SLAM_THREAD_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <QtGui>
#include <Eigen/Dense>
#include <src/plot.h>

#include "../core.h"


using namespace std;
using namespace Eigen;

class SLAMWrapper : public QThread {
Q_OBJECT

public:
    enum RunMode {
        /// Move along waypoints
        SLAM_WAYPOINT,
        /// User interactive
        SLAM_INTERACTIVE
    };

    SLAMWrapper(Conf *conf, Plot *plot, QObject *parent = 0);

    ~SLAMWrapper();

    void stop(void);

    /// Set run mode
    void setRunMode(RunMode mode);
    /// Set map filename
    void loadMap();

signals:
    void replot();
    void showMessage(QString msg);

public slots:
    virtual void commandRecieve(int command);

protected:
    virtual void run() = 0;

    void initializeLandmarkIdentifiers();

    /// Loads information and performs initial adjustment on the plot
    void configurePlot();
    /// Adds the waypoints and landmarks to the plot
    void addWaypointsAndLandmarks();
    /// Adjusts the range of the plot to show all information
    void setPlotRange();

    /**
     *     Integrate steering to predict control
     *
     *     @return -1 if there are no more waypoints, 0 if there are no commands, otherwise 1.
     */
    int control();

    /**
     * Available commands:
     *   - 1 - Forward
     *   - 2 - Backward
     *   - 3 - Turn Left
     *   - 4 - Turn Right
     *
     *   @return command value according to above list
     */
    int getCurrentCommand();

    Conf *conf;
    Plot *plot;

    /// Is finished?
    int isAlive;

    int commandId;
    /// Timestamp when command was recieved
    uint64_t commandTime;
    RunMode runMode;

    QString plotMessage;

    /// Landmark positions
    MatrixXf landmarks;
    /// Waypoints
    MatrixXf waypoints;

    int currentIteration = 0;

    // @TODO need to find a description for this item
    float Vn;
    // @TODO need to find a description for this item
    float Gn;
    /// Velocity
    float V;
    /// Steer angle
    float G;
    int nLoop;
    /// Index to first waypoint
    int indexOfFirstWaypoint = 0;
    /// True position
    VectorXf xTrue;
    /// Predicted position
    VectorXf xEstimated;
    // @TODO need to find a description for this item
    MatrixXf Q;
    // @TODO need to find a description for this item
    MatrixXf R;
    // @TODO need to find a description for this item
    MatrixXf Qe;
    // @TODO need to find a description for this item
    MatrixXf Re;
    /// Heading uncertainty, in radians
    float sigmaPhi;
    /// Change in time between predicts
    float dt;
    /// Change in time since last observation
    float dtSum = 0;
    int drawSkip;
    /// Identifier for each landmark
    vector<int> landmarkIdentifiers;
    MatrixXf plines; // Old comment: will later change to list of point

    /// Range and bearing for each of the observed landmarks
    vector<VectorXf> landmarksRangeBearing;
};

#endif // SLAM_THREAD_H
