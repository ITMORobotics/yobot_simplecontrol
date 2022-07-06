#include "servo.h"

Servo::Servo(float dt, RobotModel & model): dt(dt), model(model){
    jacobian = Eigen::MatrixXd::Zero(6, model.getDim());

    current_translation = Eigen::VectorXd::Zero(3);
    current_orient = Eigen::MatrixXd::Identity(3, 3);

    pose_err = Eigen::VectorXd::Zero(3);
    orient_err = Eigen::MatrixXd::Identity(3, 3);
    cart_twist_err = Eigen::VectorXd::Zero(6);
}

bool Servo::speedL_to_speedJ(Eigen::VectorXd & current_jpose, Eigen::VectorXd & target_speed_cart, Eigen::VectorXd & output_jvel){
    model.calcJacobian(current_jpose, jacobian);
    output_jvel = pseudoinverse(jacobian) * target_speed_cart;
    return true;
}
bool Servo::servoL_to_speedL(PID & pid, Eigen::VectorXd & current_jpose, Eigen::VectorXd & target_translation, Eigen::MatrixXd & target_orient, Eigen::VectorXd & output_speed_cart){
    model.fk(current_jpose, current_translation, current_orient);
    pose_err = target_translation - current_translation;
    Eigen::MatrixXd tmp_orient = target_orient*current_orient.transpose();
    orient_err = logmapSO3(tmp_orient);
    cart_twist_err<<pose_err, orient_err;
    std::cout<<cart_twist_err<<std::endl;
    pid.control_step(cart_twist_err);
    output_speed_cart = pid.u;
    return true;
}
