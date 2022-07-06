
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>

#include <iostream>

#include "robot_model.h"

int main(int argc, char **argv) {
	std::string urdf_filename = "../robot_description/youbot.urdf";
	std::string base_link = "base_link";
	std::string tool_link = "gripper_palm_link";

    RobotModel robot_model = RobotModel(urdf_filename, base_link, tool_link);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 6);
    Eigen::VectorXd q(5);

    Eigen::VectorXd translation(3);
    Eigen::MatrixXd orient = Eigen::MatrixXd::Zero(3, 3);

    q << 1.0, 1.0, 1.0, 1.0, 1.0;
    robot_model.calcJacobian(q, jacobian);
    std::cout<<"\n" << jacobian << std::endl;

    robot_model.fk(q, translation, orient);
    std::cout << translation << std::endl;
    std::cout << orient << std::endl;
	std::cout << "OK" << std::endl;
}
