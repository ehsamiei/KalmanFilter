#include<iostream>
#include<vector>
#include "Eigen/Dense"
#include "kalman.hpp"
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;


int main(){
    double dt{0.01}, t0{0}, t{10};
    vector<double>time;
    vector<double>PosEst;
    
    int n{2}, m{1}, p{1};   // n: states m: outputs p:control inputs
    Mat Ad(n,n), Bd(n,1), Cd(m,n), Q(n,n), R(1,1);

    Vec _x(n), _u(p), _y(m);
    Mat _P(n,n);

    Ad<<1.0, dt, 0.0, 1.0;
    Bd<< 0.5*dt*dt, dt;
    Cd<<1.0 , 0.0;

    Q << .05, .0, .0, .05;
    R << 0.5;
    _P=Eigen::MatrixXd::Zero(n,n);  

    _x<<0.0 ,0.1;
    _u<<0.0;

    vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
    };

    Kalmanfilter KF(Ad,Bd,Cd,Q,R);
    KF.kalmanTimeInit(t0,t,dt);
    KF.kalmanStateInit(_x,_u,_P);

    

    for(int i=0;i<measurements.size();i++){
      _y <<measurements[i];
      time.push_back(i*dt);
      KF.PredictionUpdate(_y);
      PosEst.push_back(KF.getter());  
    }

    plt::title("Kalman filter for the first integrator system");
    plt::plot(time,measurements,"r--");
    plt::plot(time,PosEst);
    plt::legend();
    plt::show();

    return 0;
}
