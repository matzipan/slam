#ifndef SLAM_THREAD_H
#define SLAM_THREAD_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <QtGui>
#include <Eigen/Dense>

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

    SLAMWrapper(QObject *parent = 0);

    ~SLAMWrapper();

    void stop(void);

    /**
     * Available commands:
     *   - 1 - Forward
     *   - 2 - Backward
     *   - 3 - Turn Left
     *   - 4 - Turn Right
     */
    void getCommand(int *command);

    /// Set run mode
    void setRunMode(RunMode mode);
    /// Set map filename
    void setMap(std::string &fname);


signals:
    void replot();
    void showMessage(QString msg);

public slots:
    virtual void commandRecieve(int command);

protected:
    virtual void run() = 0;

    /// Is finished?
    int isAlive;

    int commandId;
    /// Timestamp when command was recieved
    uint64_t commandTime;
    RunMode runMode;

    std::string map;

    /// Landmark positions
    MatrixXf landmarks;
    /// Waypoints
    MatrixXf waypoints;
};

#endif // SLAM_THREAD_H
