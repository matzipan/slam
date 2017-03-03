#ifndef __FASTSLAM_1_H__
#define __FASTSLAM_1_H__

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include <Eigen/Dense>

#include "core.h"
#include "slamwrapper.h"
#include "algorithms/fastslam1.h"
#include "ParticleSLAMWrapper.h"

using namespace std;
using namespace Eigen;

class FastSLAM1Wrapper : public ParticleSLAMWrapper {

public:
    FastSLAM1Wrapper(Conf *conf, NetworkPlot *plot);

    ~FastSLAM1Wrapper();

    void run();

protected:

    FastSLAM1* algorithm;
};

#endif // __FASTSLAM_1_H__
