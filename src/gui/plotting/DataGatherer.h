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

    void landmarksObservedCount(long observedCount);

    void averageLandmarkLength(float d);

private:
    double truePositionX;
    double truePositionY;
    double truePositionT;

    double estimatedPositionX;
    double estimatedPositionY;
    double estimatedPositionT;

    std::vector<double> errors;
    std::vector<double> estimatedPositionsX;
    std::vector<double> estimatedPositionsY;
    std::vector<double> truePositionsX;
    std::vector<double> truePositionsY;

    std::string simulationName;
    std::vector<uint32_t> times;
    std::vector<long> observedCounts;
    std::vector<float> averageLengthLandmark;


    void outputErrorsStats(std::vector<double> &v, std::ostream &output);

    void outputTimesStats(std::vector<uint32_t> &v, std::ostream &output);

    int turn=0;
};


#endif //SLAM_DATAGATHERER_H
