//
// Created by matzipan on 21/03/17.
//

#ifndef SLAM_DATAGATHERER_H
#define SLAM_DATAGATHERER_H

#include <string>
#include <vector>

class DataGatherer {
public:
    DataGatherer();
    ~DataGatherer();

    void setSimulationName(std::string name);
    void saveData();
    void setCarTruePosition(double x, double y, double t);
    void setCarEstimatedPosition(double x, double y, double t);
    void nextTurn();

    void cleanup();

    void loopTime(uint32_t time);

private:
    double truePositionX;
    double truePositionY;
    double truePositionT;

    double estimatedPositionX;
    double estimatedPositionY;
    double estimatedPositionT;

    std::vector<double> errors;
    std::string simulationName;
    std::vector<uint32_t> times;

    void outputErrorsStats(std::vector<double> &v, std::ostream &output);

    void outputTimesStats(std::vector<uint32_t> &v, std::ostream &output);

    int turn;
};


#endif //SLAM_DATAGATHERER_H
