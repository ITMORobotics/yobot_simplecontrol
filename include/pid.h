#include <Eigen/Eigen>
#include <Eigen/Core>

class PID{
    public:
        Eigen::MatrixXd p;
        Eigen::MatrixXd i;
        Eigen::MatrixXd d;

        Eigen::VectorXd u;

        PID(Eigen::MatrixXd & p, Eigen::MatrixXd & i, Eigen::MatrixXd & d, float dt);
        // PID(double p, double i, double d);
        
        void control_step(Eigen::VectorXd error);
        void reset();

    private:
        float dt;
        uint dim;
        Eigen::VectorXd last_error;
        Eigen::VectorXd integral_value;
};