#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>
#include <iostream>
#include <math.h>

#include "pid.h"

int main(int argc, char **argv) {

    float dt = 0.01;
    Eigen::MatrixXd P = 0.5*Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd I = 1e-4*Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd D = 0.002*Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd err = Eigen::VectorXd::Zero(6);

    PID pid(P,I,D, dt);
    for(int i=0; i<1e3; i++){
        err = Eigen::VectorXd::Ones(6)*sin(2*i*dt);
        pid.control_step(err);
        std::cout<< pid.u<<"\n"<< std::endl;
    }
}
