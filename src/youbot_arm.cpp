#include "youbot_arm.h"
#include <chrono>
#include <thread>
#include "math.h"


YoubotArm::YoubotArm(const std::string name, const std::string configFilePath, float dt = 0.002, uint nj = 5):
	dt(dt), robot_state(nj)
{
	
    try {
		youbot_ethcat = new youbot::YouBotManipulator(name, configFilePath);
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		exit(1);
	}
}

YoubotArm::~YoubotArm(){
	delete youbot_ethcat;
	youbot_ethcat = 0;
}

bool YoubotArm::init(){
	youbot_ethcat->doJointCommutation();
	youbot_ethcat->calibrateManipulator();
	return true;
}

bool YoubotArm::update_state(){
	std::vector<youbot::JointSensedAngle> posedata;
	std::vector<youbot::JointSensedVelocity> veldata;

	youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
	youbot_ethcat->getJointData(posedata);
	youbot_ethcat->getJointData(veldata);
	youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

	// std::cout<<robot_state.njoints<<std::endl;
	for(int i = 0; i<robot_state.njoints; i++){
		robot_state.q(i) = posedata[i].angle.value();
		robot_state.dq(i) = veldata[i].angularVelocity.value();
	}

	return true;

}

bool YoubotArm::speedj(Eigen::VectorXd & jvel){
	auto start = chrono::system_clock::now().time_since_epoch();
	bool ok = update_state();
	std::vector<youbot::JointVelocitySetpoint> jvel_youbot;
	for(int i=0; i<jvel.size(); i++){
		youbot::JointVelocitySetpoint desiredAngularVelocity;
		desiredAngularVelocity.angularVelocity = jvel(i) * boost::units::si::radian_per_second;
		jvel_youbot.push_back(desiredAngularVelocity);
	}
	youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
	youbot_ethcat->setJointData(jvel_youbot);
	youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
	
	auto end = chrono::system_clock::now().time_since_epoch();
    auto endles_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    auto sleep_for = (long)fmax(0, dt*1e6 - endles_time);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_for));

	return ok;
}

