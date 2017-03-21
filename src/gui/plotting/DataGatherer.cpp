//
// Created by matzipan on 21/03/17.
//

#include <cmath>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <iostream>
#include "DataGatherer.h"

DataGatherer::DataGatherer() {}

DataGatherer::~DataGatherer() {}

void DataGatherer::setSimulationName(std::string name) {
    simulationName = name;
}

void DataGatherer::saveData() {
    std::ofstream out(simulationName + "_results.txt");

    double sum = std::accumulate(errors.begin(), errors.end(), 0.0);
    double mean = sum / errors.size();

    double sq_sum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / errors.size() - mean * mean);

    double minimum = *std::min_element(errors.begin(), errors.end());
    double maximum = *std::max_element(errors.begin(), errors.end());


    out<<"Stats:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum;
    std::cout<<"Stats:\nMean: "<<mean<<" Std: "<<stdev<<" Min: "<<minimum<<" Max: "<<maximum;

    out.close();
    errors.clear();
}

void DataGatherer::nextTurn() {
    errors.push_back(sqrt(pow(truePositionX-estimatedPositionX, 2) + pow(truePositionY-estimatedPositionY, 2)));
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
