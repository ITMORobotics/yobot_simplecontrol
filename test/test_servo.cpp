#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>
#include <iostream>
#include <math.h>

#include "servo.h"


int main(int argc, char **argv) {

    float dt = 0.01;
    Eigen::MatrixXd P = 0.5*Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd I = 1e-4*Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd D = 0.002*Eigen::MatrixXd::Identity(6, 6);

	std::string urdf_filename = "../robot_description/youbot.urdf";
	std::string base_link = "base_link";
	std::string tool_link = "gripper_palm_link";

    RobotModel robot_model = RobotModel(urdf_filename, base_link, tool_link);
    PID pid(P,I,D, dt);

    Servo servo(dt, robot_model);
    Eigen::VectorXd output_speed_cart = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd current_jpose = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd target_translation(3);
    Eigen::MatrixXd target_orient = Eigen::MatrixXd::Zero(3, 3);

    Eigen::VectorXd jvel = Eigen::VectorXd::Zero(5);


    for(int i=0; i<1e3; i++){
        current_jpose = Eigen::VectorXd::Ones(5)*sin(2*i*dt);
        target_translation<< 0.2, 0.2, 0.45;
        servo.servoL_to_speedL(pid, current_jpose, target_translation, target_orient, output_speed_cart);
        servo.speedL_to_speedJ(current_jpose, output_speed_cart, jvel);
        std::cout<<jvel<<"\n"<<std::endl;
    }
}
