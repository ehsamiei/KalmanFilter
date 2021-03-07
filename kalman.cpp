#include "kalman.hpp"

Kalmanfilter::Kalmanfilter(Mat Ad, Mat Bd,Mat Cd,
                           Mat Q, Mat R): Ad(Ad), 
                           Bd(Bd), Cd(Cd), Q(Q), R(R){
                            n=Ad.rows(),m=Cd.rows(),p=Bd.cols();    
                           } 

void Kalmanfilter::kalmanTimeInit(double _t0,double _t, double _dt) {
    t0=_t0;
    t=_t;
    dt=_dt;
}
void Kalmanfilter::kalmanStateInit(Vec _x, Vec _u, Mat _P){
    x=_x;
    u=_u;
    P=_P;
}    

void Kalmanfilter::PredictionUpdate(Vec _y){
    
    //Prediction
    x=Ad*x+Bd*u;
    P=Ad*P*Ad.transpose()+Q;
    //Update
    y=_y;
    S=Cd*P*Cd.transpose()+R;
    K=P*Cd.transpose()*S.inverse();
    x=x+K*(y-Cd*x);
}

double Kalmanfilter::getter(){
    return x[0];
}



