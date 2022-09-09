#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>
#include <iostream>
#include <math.h>

#include "servo.h"
#include "youbot_arm.h"


#define dt 0.002

std::string urdf_filename = "../robot_description/youbot.urdf";
std::string base_link = "base_link";
std::string tool_link = "gripper_palm_link";

Eigen::VectorXd candle_joint_position(5);
Eigen::VectorXd hold_joint_torque(5);

JointState target_state(5);

double m3 = 0.934;
double r3 = 0.185;

double k4 = -3.0;
double m4 = 0.251 + 0.75;
double r4 = 0.130;



int main(int argc, char **argv) {
    // Setting up first position in candle
    candle_joint_position<< 2.9496, 1.1344, -2.5482, 1.789, 2.9234;
    hold_joint_torque = Eigen::VectorXd::Zero(5);

    target_state.q = candle_joint_position;
    
    VectorJointControlType joint_multitype = {JointPosition, JointPosition, JointPosition, JointTorque, JointPosition};
    
    YoubotArm arm = YoubotArm("youbot",  "/media/files/projects/youbot_painter/submodules/youbot_driver/config", dt, 5);
    arm.init();

    for(int i=0; i< 20000; i++){
        arm.update_state();
        hold_joint_torque[3] = k4*m4*r4*sin(arm.joint_state.q[3] - candle_joint_position[3]);
        
        target_state.torq = hold_joint_torque;
        std::cout<< arm.joint_state.q - candle_joint_position<< "\n"<<std::endl;
        arm.servo_joint_state(joint_multitype, target_state);
    }

    std::cout << "OK" << std::endl;
}
