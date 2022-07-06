#include <Eigen/Eigen>
#include <Eigen/Core>

#include "robot_model.h"
#include "pid.h"

class Servo{
    public:
        Servo(float dt, RobotModel & model);

        bool speedL_to_speedJ(Eigen::VectorXd & current_jpose, Eigen::VectorXd & target_speed_cart, Eigen::VectorXd & output_jvel);
        bool servoL_to_speedL(PID & pid, Eigen::VectorXd & current_jpose, Eigen::VectorXd & target_translation, Eigen::MatrixXd & target_orient, Eigen::VectorXd & output_speed_cart);
    private:
        RobotModel model;
        float dt;

        Eigen::MatrixXd jacobian;

        Eigen::VectorXd current_translation;
        Eigen::MatrixXd current_orient;

        Eigen::VectorXd pose_err;
        Eigen::MatrixXd orient_err;
        Eigen::VectorXd cart_twist_err;
};