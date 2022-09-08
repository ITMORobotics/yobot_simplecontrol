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
    target_speed_cart(0) = -0.01;

    Eigen::VectorXd init_jpose(5);
    init_jpose<< M_PI/2, M_PI/2, -M_PI/2, M_PI, M_PI;

    Eigen::VectorXd jvel = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd jtorq = Eigen::VectorXd::Zero(5);

    Eigen::VectorXd target_translation(3);
    target_translation << 0.2, 0.1, 0.1;
    Eigen::MatrixXd target_orient = Eigen::MatrixXd::Zero(3, 3);

    VectorJointControlType joint_multitype = {JointPosition, JointPosition, JointPosition, JointPosition, JointTorque};
    JointState target_joint_state(5);
    target_joint_state.q = init_jpose;
    target_joint_state.torq = jtorq;
    
    YoubotArm arm = YoubotArm("youbot",  "/media/files/projects/youbot_painter/submodules/youbot_driver/config", dt, 5);
    arm.init();

    // Test servoj
    for(int i=0; i< 2000; i++){
        arm.update_state();
        arm.servoj(init_jpose);
        
    }
    std::cout<< arm.joint_state.q<< "\n"<<std::endl;
    std::cout << "servoj finished" << std::endl;
    
    // Test speedl
    for(int i=0; i< 2000; i++){
        arm.update_state();
        
        // servo.servoL_to_speedL(pid, arm.joint_state.q, target_translation, target_orient, target_speed_cart);
        servo.speedL_to_speedJ(arm.joint_state.q, target_speed_cart, jvel);
        arm.speedj(jvel);
    }
    jvel = Eigen::VectorXd::Zero(5);
    arm.speedj(jvel);
    std::cout<< arm.joint_state.q<< "\n"<<std::endl;
    std::cout << "speedj finished" << std::endl;

    // Test speedj with rorqj in diffrent joints
    for(int i=0; i< 2000; i++){
        arm.update_state();
        arm.servo_joint_state(joint_multitype ,target_joint_state);
    }
    std::cout<< arm.joint_state.torq<< "\n"<<std::endl;
    std::cout << "servo_joint_state finished" << std::endl;

    // for(int i=0; i< 1000; i++){
    //     arm.update_state();
    //     arm.torqj(jtorq);
    // }
    // std::cout<< arm.joint_state.torq<< "\n"<<std::endl;
    // std::cout << "torqj finished" << std::endl;

    jvel = Eigen::VectorXd::Zero(5);
    arm.speedj(jvel);

    std::cout << "OK" << std::endl;
}
