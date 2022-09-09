#include "youbot_arm.h"
#include <chrono>
#include <thread>
#include "math.h"


YoubotArm::YoubotArm(const std::string name, const std::string configFilePath, float dt = 0.002, uint nj = 5):
	dt(dt), joint_state(nj)
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
	std::cout<<"Robot disconnected"<<std::endl;
}

bool YoubotArm::init(){
	youbot_ethcat->doJointCommutation();
	youbot_ethcat->calibrateManipulator();
	return true;
}

bool YoubotArm::update_state(){
	std::vector<youbot::JointSensedAngle> posedata;
	std::vector<youbot::JointSensedVelocity> veldata;
	std::vector<youbot::JointSensedTorque> torqdata;

	youbot_ethcat->getJointData(posedata);
	youbot_ethcat->getJointData(veldata);
	youbot_ethcat->getJointData(torqdata);

	// std::cout<<joint_state.njoints<<std::endl;
	for(int i = 0; i<joint_state.njoints; i++){
		joint_state.q(i) = posedata[i].angle.value();
		joint_state.dq(i) = veldata[i].angularVelocity.value();
		joint_state.torq(i) = torqdata[i].torque.value();
	}
	return true;
}

bool YoubotArm::servoj(Eigen::VectorXd & jpose){
	auto start = chrono::system_clock::now().time_since_epoch();
	bool ok = update_state();
	std::vector<youbot::JointAngleSetpoint> jpose_youbot;
	for(int i=0; i<jpose.size(); i++){
		youbot::JointAngleSetpoint desiredAngle;
		desiredAngle.angle = jpose(i) * boost::units::si::radian;
		jpose_youbot.push_back(desiredAngle);
	}
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
	youbot_ethcat->setJointData(jpose_youbot);
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
	
	auto end = chrono::system_clock::now().time_since_epoch();
    auto endles_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    auto sleep_for = (long)fmax(0, dt*1e6 - endles_time);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_for));

	return ok;
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
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
	youbot_ethcat->setJointData(jvel_youbot);
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
	
	auto end = chrono::system_clock::now().time_since_epoch();
    auto endles_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    auto sleep_for = (long)fmax(0, dt*1e6 - endles_time);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_for));

	return ok;
}

bool YoubotArm::torqj(Eigen::VectorXd & jtorq){
	auto start = chrono::system_clock::now().time_since_epoch();
	bool ok = update_state();
	std::vector<youbot::JointTorqueSetpoint> jtorq_youbot;
	for(int i=0; i<jtorq.size(); i++){
		youbot::JointTorqueSetpoint desiredTorque;
		desiredTorque.torque = jtorq(i) * boost::units::si::newton_meter;
		jtorq_youbot.push_back(desiredTorque);
	}
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
	youbot_ethcat->setJointData(jtorq_youbot);
	// youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
	
	auto end = chrono::system_clock::now().time_since_epoch();
    auto endles_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    auto sleep_for = (long)fmax(0, dt*1e6 - endles_time);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_for));

	return ok;
}

bool YoubotArm::servo_joint_state(VectorJointControlType & joint_state_control_types, JointState & joint_state){
	auto start = chrono::system_clock::now().time_since_epoch();
	bool ok = update_state();
	youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
	for(int i=0; i<joint_state.njoints; i++){
		switch (joint_state_control_types[i])
		{
		case JointPosition: {
			youbot::JointAngleSetpoint jointData = joint_state.q[i] * boost::units::si::radian;
			youbot_ethcat->getArmJoint(i+1).setData(jointData);
			break;
		}
		case JointSpeed: {
			youbot::JointVelocitySetpoint jointData = joint_state.dq[i] * boost::units::si::radian_per_second;
			youbot_ethcat->getArmJoint(i+1).setData(jointData);
			break;
		}
		case JointTorque: {
			youbot::JointTorqueSetpoint jointData = joint_state.torq[i] * boost::units::si::newton_meter;
			youbot_ethcat->getArmJoint(i+1).setData(jointData);
			break;
		}
		default:
			break;
		}
	}
	youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

	auto end = chrono::system_clock::now().time_since_epoch();
    auto endles_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    auto sleep_for = (long)fmax(0, dt*1e6 - endles_time);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_for));

	return ok;
}



