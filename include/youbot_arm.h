#include "robot.h"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

enum JointControlType
{
    JointPosition,
    JointSpeed,
    JointTorque
};

typedef std::vector<JointControlType> VectorJointControlType;

class YoubotArm{
    private:
        youbot::YouBotManipulator* youbot_ethcat;
        float dt;
    public:
        JointState joint_state;

        YoubotArm(const std::string name, const std::string configFilePath, float dt, uint njoints);
        ~YoubotArm();
        bool init();
        bool update_state();
        bool servoj(Eigen::VectorXd & jpose);
        bool speedj(Eigen::VectorXd & jvel);
        bool torqj(Eigen::VectorXd & jtorq);
        bool servo_joint_state(VectorJointControlType & joint_state_control_types, JointState & joint_state);
};