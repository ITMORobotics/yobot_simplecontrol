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

	std::string urdf_filename = "../robot_description/youbot.urdf";
	std::string base_link = "base_link";
	std::string tool_link = "gripper_palm_link";

    RobotModel robot_model = RobotModel(urdf_filename, base_link, tool_link);
    
    YoubotArm arm = YoubotArm("youbot",  "/media/files/projects/youbot_painter/submodules/youbot_driver/config", 0.002, 5);
    arm.init();

    for(int i=0; i< 1000; i++){
        arm.update_state();
        std::cout<< arm.robot_state.q<< "\n"<<std::endl;
        
    }
}
