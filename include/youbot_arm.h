#include "robot.h"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

class YoubotArm{
    private:
        youbot::YouBotManipulator* youbot_ethcat;
        float dt;
    public:
        RobotState robot_state;

        YoubotArm(const std::string name, const std::string configFilePath, float dt, uint njoints);
        ~YoubotArm();
        bool init();
        bool update_state();
        // bool servoj(Eigen::VectorXd & jpose) override;
        bool speedj(Eigen::VectorXd & jvel);
        // bool speedl(Eigen::VectorXd & translation, Eigen::MatrixXd & orient) override;
};