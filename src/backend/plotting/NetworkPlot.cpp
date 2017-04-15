//
// Created by matzipan on 06/02/17.
//

#include "NetworkPlot.h"

NetworkPlot::NetworkPlot() {
    context = new zmqpp::context();

    socket = new zmqpp::socket(*context, zmqpp::socket_type::pair);

    socket->connect("tcp://127.0.0.1:4242");

    printf("Connected to plotting server\n");
}

NetworkPlot::~NetworkPlot() {
    socket->close();
    context->terminate();
}

void NetworkPlot::sendXYArrays(zmqpp::message &message, std::vector<double> &xs, std::vector<double> &ys) {
    message<<(int32_t) xs.size();
    for (int32_t i = 0; i < (int32_t) xs.size(); i++) {
        message << xs[i];
    }

    message<<(int32_t) ys.size();
    for (int32_t i = 0; i < (int32_t) ys.size(); i++) {
        message << ys[i];
    }

    socket->send(message);
}

void NetworkPlot::setLandmarks(std::vector<double> &xs, std::vector<double> &ys) {
    zmqpp::message message;

    message<<"setLandmarks";

    sendXYArrays(message, xs, ys);
}

void NetworkPlot::setWaypoints(std::vector<double> &xs, std::vector<double> &ys) {
    zmqpp::message message;

    message<<"setWaypoints";

    sendXYArrays(message, xs, ys);
}

void NetworkPlot::setParticles(std::vector<double> &xs, std::vector<double> &ys) {
    zmqpp::message message;

    message<<"setParticles";

    sendXYArrays(message, xs, ys);
}

void NetworkPlot::setFeatureParticles(std::vector<double> &xs, std::vector<double> &ys) {
    zmqpp::message message;

    message<<"setFeatureParticles";

    sendXYArrays(message, xs, ys);
}

void NetworkPlot::setLaserLines(Eigen::MatrixXf &lnes) {
    zmqpp::message message;

    message<<"setLaserLines";

    message<<(uint32_t) lnes.rows()<<(uint32_t) lnes.cols();
    for (int i = 0; i < lnes.rows(); i++) {
        for (int j = 0; j < lnes.cols(); j++) {
            message << lnes(i, j);
        }
    }

    socket->send(message, true);
}

void NetworkPlot::setCovEllipse(Eigen::MatrixXf &lnes, int idx) {
    zmqpp::message message;

    message<<"setCovEllipse";

    message<<(uint32_t) lnes.rows()<<(uint32_t) lnes.cols();
    for (int i = 0; i < lnes.rows(); i++) {
        for (int j = 0; j < lnes.cols(); j++) {
            message << lnes(i, j);
        }
    }

    message<<idx;

    socket->send(message, true);
}

void NetworkPlot::addTruePosition(double x, double y) {
    zmqpp::message message;

    message<<"addTruePosition";

    message<<x<<y;

    socket->send(message, true);
}

void NetworkPlot::addEstimatedPosition(double x, double y) {
    zmqpp::message message;

    message<<"addEstimatedPosition";

    message<<x<<y;

    socket->send(message, true);
}

void NetworkPlot::setCarSize(double s, uint32_t id) {
    zmqpp::message message;

    message<<"setCarSize";

    message<<s<<id;

    socket->send(message, true);
}

void NetworkPlot::setCarTruePosition(double x, double y, double t) {
    zmqpp::message message;

    message<<"setCarTruePosition";

    message<<x<<y<<t;

    socket->send(message, true);
}

void NetworkPlot::setCarEstimatedPosition(double x, double y, double t) {
    zmqpp::message message;

    message<<"setCarEstimatedPosition";

    message<<x<<y<<t;

    socket->send(message, true);
}

void NetworkPlot::setPlotRange(double xmin, double xmax, double ymin, double ymax) {
    zmqpp::message message;

    message<<"setPlotRange";

    message<<xmin<<xmax<<ymin<<ymax;

    socket->send(message, true);
}

void NetworkPlot::clear() {
    zmqpp::message message;

    message<<"clear";

    socket->send(message, true);
}

void NetworkPlot::setSimulationName(std::string filename) {
    zmqpp::message message;

    message<<"setSimulationName"<<filename;

    socket->send(message, true);
}

void NetworkPlot::setCurrentIteration(uint32_t iteration) {
    if(true) {
        // TODO: this function triggers a race condition with zeroMQ and haven't been able to track it down, so I'm just gonna disable it
        return;
    }
    zmqpp::message message;

    message<<"setCurrentIteration"<<iteration;

    socket->send(message);
}

void NetworkPlot::plot() {
    zmqpp::message message;

    message<<"plot";

    socket->send(message, true);
}

void NetworkPlot::covEllipseAdd(uint32_t n) {
    zmqpp::message message;

    message<<"covEllipseAdd"<<n;

    socket->send(message, true);
}

void NetworkPlot::endPlot() {
    zmqpp::message message;

    message<<"endPlot";

    socket->send(message, true);
}

void NetworkPlot::loopTime(uint32_t time) {
    zmqpp::message message;

    message<<"loopTime"<<time;

    socket->send(message, true);
}

/*// @TODO These unimplemented functions can be used to send messages from WindowPlot to NetworkPlot when an event happens
void NetworkPlot::canvasMousePressEvent(QMouseEvent *event) {}
void NetworkPlot::keyPressEvent(QKeyEvent *event) {}
void NetworkPlot::mousePressEvent(QMouseEvent *event) {}*/
