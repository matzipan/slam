//
// Created by matzipan on 21/03/17.
//

#include <cmath>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <sys/stat.h>
#include <iomanip>
#include "DataGatherer.h"

DataGatherer::DataGatherer() {}

DataGatherer::~DataGatherer() {}

void DataGatherer::setSimulationName(std::string name) {
    simulationName = name;
}

void DataGatherer::outputErrorsStats(std::vector<double> &v, std::ostream &output) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);

    double minimum = *std::min_element(v.begin(), v.end());
    double maximum = *std::max_element(v.begin(), v.end());

    output<<"Errors:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum<<"\n";
    std::cout<<"Errors:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum<<"\n";
}

void DataGatherer::outputTimesStats(std::vector<uint32_t> &v, std::ostream &output) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);

    double minimum = *std::min_element(v.begin(), v.end());
    double maximum = *std::max_element(v.begin(), v.end());

    output<<"Times:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum<<"\n";
    std::cout<<"Times:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum<<"\n";
}

void DataGatherer::saveData() {
    mkdir(simulationName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    std::ofstream resultsSynthesisOutput(simulationName + "/results.txt");
    std::ofstream errorsOutput(simulationName + "/errors.txt");
    std::ofstream timesOutput(simulationName + "/times.txt");
    std::ofstream positionsOutput(simulationName + "/positions.txt");
    std::ofstream observedCountsOutput(simulationName + "/observedCounts.txt");
    std::ofstream averageLengthLandmarkOutput(simulationName + "/averageLengthLandmark.txt");

    outputErrorsStats(errors, resultsSynthesisOutput);
    outputTimesStats(times, resultsSynthesisOutput);

    for(double x : errors) {
        errorsOutput<<std::setprecision(10)<<x<<"\n";
    }

    for(uint32_t x : times) {
        timesOutput<<std::setprecision(10)<<x<<"\n";
    }

    for(long x : observedCounts) {
        observedCountsOutput<<x<<"\n";
    }


    for(float x : averageLengthLandmark) {
        averageLengthLandmarkOutput<<x<<"\n";
    }

    for(int i = 0; i < estimatedPositionsX.size(); i++) {
        positionsOutput<<std::setprecision(10)<<truePositionsX[i]<<", "<<truePositionsY[i]<<", "<<estimatedPositionsX[i]<<", "<<estimatedPositionsY[i]<<"\n";
    }

    resultsSynthesisOutput.close();
    errorsOutput.close();
    timesOutput.close();
    positionsOutput.close();
    observedCountsOutput.close();
    averageLengthLandmarkOutput.close();
}

void DataGatherer::cleanup() {
    errors.clear();
    times.clear();
    estimatedPositionsX.clear();
    estimatedPositionsY.clear();
    truePositionsX.clear();
    truePositionsY.clear();
    observedCounts.clear();
    averageLengthLandmark.clear();
}

void DataGatherer::nextTurn() {
    errors.push_back(sqrt(pow(truePositionX-estimatedPositionX, 2) + pow(truePositionY-estimatedPositionY, 2)));
    estimatedPositionsX.push_back(estimatedPositionX);
    estimatedPositionsY.push_back(estimatedPositionY);
    truePositionsX.push_back(truePositionX);
    truePositionsY.push_back(truePositionY);

    turn++;

    if(turn % 100 == 0) {
        saveData();
    }
}

void DataGatherer::setCarTruePosition(double x, double y, double t) {
    truePositionX = x;
    truePositionY = y;
    truePositionT = t;
}

void DataGatherer::setCarEstimatedPosition(double x, double y, double t) {
    estimatedPositionX = x;
    estimatedPositionY = y;
    estimatedPositionT = t;
}

void DataGatherer::loopTime(uint32_t time) {
    times.push_back(time);
}

void DataGatherer::landmarksObservedCount(long observedCount) {
    observedCounts.push_back(observedCount);
}

void DataGatherer::averageLandmarkLength(float average) {
    averageLengthLandmark.push_back(average);
}
