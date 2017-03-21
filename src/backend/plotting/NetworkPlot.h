//
// Created by matzipan on 06/02/17.
//

#ifndef SLAM_GUI_NETWORKPLOT_H
#define SLAM_GUI_NETWORKPLOT_H

#include <Eigen/Dense>
#include <zmqpp.hpp>


class NetworkPlot {

public:
    NetworkPlot();

    ~NetworkPlot();

    void setLandmarks(std::vector<double> &xs, std::vector<double> &ys);
    void setWaypoints(std::vector<double> &x, std::vector<double> &ys);
    void setParticles(std::vector<double> &x, std::vector<double> &ys);
    void setFeatureParticles(std::vector<double> &x, std::vector<double> &ys);
    void setLaserLines(Eigen::MatrixXf &lnes);
    void setCovEllipse(Eigen::MatrixXf &lnes, int idx);
    void addTruePosition(double x, double y);
    void addEstimatedPosition(double x, double y);
    void setCarSize(double s, int id = 0);
    void setCarTruePosition(double x, double y, double t);
    void setCarEstimatedPosition(double x, double y, double t);
    void setPlotRange(double xmin, double xmax, double ymin, double ymax);
    void clear(void);
    void setSimulationName(std::string filename);


/*public slots:*/

    void plot();
    void endPlot();
    void setCurrentIteration(int iteration);
    void covEllipseAdd(int n);

/*    void canvasMousePressEvent(QMouseEvent *event);

    signals:

    void commandSend(int cmd);
    void addCovEllipse(int n);*/


protected:
    zmqpp::socket *socket;
    zmqpp::context *context;

    /*void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);*/

    void sendXYArrays(zmqpp::message &message, std::vector<double> &xs, std::vector<double> &ys);
};


#endif //SLAM_GUI_NETWORKPLOT_H
