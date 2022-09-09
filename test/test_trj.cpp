#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>
#include <iostream>
#include <math.h>

#include "servo.h"
#include "youbot_arm.h"
#include "csv.h"

#define dt 0.002

std::string urdf_filename = "../robot_description/youbot.urdf";
std::string base_link = "base_link";
std::string tool_link = "gripper_palm_link";

Eigen::VectorXd joint_position(5);

double l3 = 0.155;
double k3 = -3.0;
double m3 = 1.558;
double r3 = 0.185 + 0.078;

double k4 = -3.0;
double m4 = 0.251 + 0.75;
double r4 = 0.155;



int main(int argc, char **argv) {

    io::CSVReader<5> in("../trajectories/trj_1009_127.csv");
    in.read_header(io::ignore_extra_column, "q1", "q2", "q3", "q4", "q5");
    
    YoubotArm arm = YoubotArm("youbot",  "/media/files/projects/youbot_painter/submodules/youbot_driver/config", dt, 5);
    arm.init();


    double q1, q2, q3, q4, q5;
    in.read_row(q1, q2, q3, q4, q5);
    joint_position<<q1,q2,q3,q4,q5;

    for(int i=0; i< 2000; i++){
        arm.update_state();
        arm.servoj(joint_position);
    }
    
    while(in.read_row(q1, q2, q3, q4, q5)){
        joint_position<<q1,q2,q3,q4,q5;
        arm.update_state();
        arm.servoj(joint_position);
    }

    std::cout << "OK" << std::endl;
}
