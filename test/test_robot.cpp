#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>
#include <iostream>
#include <math.h>

#include "servo.h"
#include "youbot_arm.h"


int main(int argc, char **argv) {

    float dt = 0.002;

	Eigen::MatrixXd P = 1.5*Eigen::MatrixXd::Identity(6, 6);
    P.diagonal() << 1.5, 1.5, 1.5, 10.5, 10.5, 10.5;
    Eigen::MatrixXd I = 5e-4*Eigen::MatrixXd::Identity(6, 6);
    I.diagonal() << 5e-4, 5e-4, 5e-4, 2e-4, 2e-4, 2e-4;
    Eigen::MatrixXd D = 0.005*Eigen::MatrixXd::Identity(6, 6);
    D.diagonal() << 0.005, 0.005, 0.005, 0.05, 0.05, 0.05;

	std::string urdf_filename = "../robot_description/youbot.urdf";
	std::string base_link = "base_link";
	std::string tool_link = "gripper_palm_link";

    RobotModel robot_model = RobotModel(urdf_filename, base_link, tool_link);
    PID pid(P,I,D, dt);

    Servo servo(dt, robot_model);

    Eigen::VectorXd target_speed_cart = Eigen::VectorXd::Zero(6);
    // target_speed_cart(2) = -0.01;

    Eigen::VectorXd jvel = Eigen::VectorXd::Zero(5);

    Eigen::VectorXd target_translation(3);
    target_translation << 0.2, 0.1, 0.1;
    Eigen::MatrixXd target_orient = Eigen::MatrixXd::Zero(3, 3);
    
    YoubotArm arm = YoubotArm("youbot",  "/media/files/projects/youbot_painter/submodules/youbot_driver/config", dt, 5);
    arm.init();

    for(int i=0; i< 30000; i++){
        arm.update_state();
        std::cout<< arm.robot_state.q<< "\n"<<std::endl;
        servo.servoL_to_speedL(pid, arm.robot_state.q, target_translation, target_orient, target_speed_cart);
        servo.speedL_to_speedJ(arm.robot_state.q, target_speed_cart, jvel);
        arm.speedj(jvel);
    }
    jvel = Eigen::VectorXd::Zero(5);
    arm.speedj(jvel);
}
