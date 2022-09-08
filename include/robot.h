#include <string>
#include <Eigen/Eigen>
#include <Eigen/Core>

struct JointState{
    public:
        uint njoints;
        Eigen::VectorXd q;
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        Eigen::VectorXd torq;

        JointState(uint njoints){
            this->njoints = njoints;
            q = Eigen::VectorXd::Zero(njoints);
            dq = Eigen::VectorXd::Zero(njoints);
            ddq = Eigen::VectorXd::Zero(njoints);
            torq = Eigen::VectorXd::Zero(njoints);
        }

        void reset(){
            q = Eigen::VectorXd::Zero(njoints);
            dq = Eigen::VectorXd::Zero(njoints);
            ddq = Eigen::VectorXd::Zero(njoints);
            torq = Eigen::VectorXd::Zero(njoints);
        }
};

// class Robot{
//     protected:
//         std::string name;
//         std::string configFilePath;
//         float dt;
//         uint njoints;
        
//     public:
//         RobotState * robot_state;

//         Robot(const std::string name, const std::string configFilePath, float dt, uint njoints):
//             name(name), configFilePath(configFilePath), dt(dt), njoints(njoints)
//         {}
//         virtual bool init();
//         virtual bool update_state();
//         // virtual bool servoj(Eigen::VectorXd & jpose) = 0;
//         virtual bool speedj(Eigen::VectorXd & vel) = 0;
//         // virtual bool servol(Eigen::VectorXd & translation, Eigen::MatrixXd & orient) = 0;
// };