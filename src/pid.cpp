#include "pid.h"

PID::PID(Eigen::MatrixXd & p, Eigen::MatrixXd & i, Eigen::MatrixXd & d, float dt)
    : p(p), i(i), d(d), dt(dt){
    dim = d.cols();
    reset();
}

void PID::control_step(Eigen::VectorXd error){
    integral_value = error*this->dt;
    u = this->p*error + this->i*this->integral_value + this->d*(error - this->last_error)/dt;
    last_error = error;
}

void PID::reset(){
    last_error = Eigen::VectorXd::Zero(dim);
    integral_value = Eigen::VectorXd::Zero(dim);
    u = Eigen::VectorXd::Zero(dim);
}