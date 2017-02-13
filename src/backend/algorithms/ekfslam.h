//
// Created by matzipan on 11/10/16.
//

#ifndef __EKFSLAM_H__
#define __EKFSLAM_H__

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class EKFSLAM {

public:
    EKFSLAM();

    ~EKFSLAM();

    void sim(MatrixXf &landmarks, MatrixXf &waypoints, VectorXf &x, MatrixXf &P, float noisyV, float noisyG, MatrixXf &Qe,
                 float dt, float phi, vector<int> landmarkIdentifiers, vector<VectorXf> landmarkRangeBearing, MatrixXf Re,
                 bool observe, vector<VectorXf> zf, vector<int> idf, vector<VectorXf> zn, vector<int> dataAssociationTable,
                 MatrixXf R);

    bool enableBatchUpdate;
    bool useHeading;
    float wheelBase;
    float gateReject;
    float gateAugment;
    int associationKnown;
    float sigmaPhi;

protected:

    void predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float wheelBase, float dt);

    void batchUpdate(VectorXf &x, MatrixXf &P, vector<VectorXf> &zf, MatrixXf &R, vector<int> &idf);

    void observeHeading(VectorXf &x, MatrixXf &P, float phi);

    void dataAssociate(VectorXf &x, MatrixXf &P, vector<VectorXf> &landmarksRangeBearing, MatrixXf &R, float gate1, float gate2,
                       vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn);

    void dataAssociateKnown(VectorXf &x, vector<VectorXf> &landmarkRangeBearing, vector<int> &idz, vector<VectorXf> &zf,
                            vector<int> &idf, vector<VectorXf> &zn, vector<int> &dataAssociationTable);

    void augment(VectorXf &x, MatrixXf &P, vector<VectorXf> &zn, MatrixXf &Re);

    void ekfComputeAssociation(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &R, int idf, float &nis, float &nd);

    void ekfObserveModel(VectorXf &x, int idf, VectorXf &z, MatrixXf &H);

    void ekfAddOneZ(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &Re);

};


#endif // __EKFSLAM_H__
