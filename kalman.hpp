#ifndef KALMAN_H_
#define KALMAN_H_

#include<iostream>
#include<vector>
#include "Eigen/Dense"


using namespace std;


typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;

class Kalmanfilter{
public:
    Kalmanfilter(Mat Ad, Mat Bd, Mat Cd, Mat Q, Mat R);
    void kalmanTimeInit(double t0,double t,double dt);
    void kalmanStateInit(Vec x,Vec u,Mat _P);
    void PredictionUpdate(Vec y);
    double getter();

private:
    Mat Ad,Bd,Cd;
    Mat Q,R,P;
    double t0,t,dt;
    int n,m,p;
    Vec x,u,y;
    Mat S,K;

};

#endif