#include "robot_model.h"

template<typename T>
void eigen_to_kdl(Eigen::VectorXd & q, T & t){
    for(int i=0; i< q.size(); i++){
        t(i) = q[i];
    }
}

template<typename T>
void kdl_to_eigen(T & t, Eigen::VectorXd & q){
    for(int i=0; i< q.size(); i++){
        q[i] = t(i);
    }

}

RobotModel::RobotModel(std::string urdf_filename, std::string base_link, std::string tool_link):gravity(0, 0, -9.82)
	{
        if (!kdl_parser::treeFromFile(urdf_filename, tree)) {
            std::cout << "Failed to construct kdl tree\n" << std::endl;
        }
        if(!tree.getChain(base_link, tool_link, chain)){
            std::cout << "Failed to get KDL chain from tree\n" << std::endl;
        }
        dim = chain.getNrOfJoints();
        fksolverpose = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
        jacsolver =  std::make_shared<KDL::ChainJntToJacSolver>(chain);
        djacsolver =  std::make_shared<KDL::ChainJntToJacDotSolver>(chain);
	}

void RobotModel::calcJacobian(Eigen::VectorXd & q, Eigen::MatrixXd & jac)
{   
    assert(("Invalid shape of given vector q", q.size()==dim));
    KDL::JntArray jnt(dim);
    KDL::Jacobian jacobian(dim);
    eigen_to_kdl(q, jnt);
    jacsolver->JntToJac(jnt, jacobian);
    jac = jacobian.data;
}


void RobotModel::fk(Eigen::VectorXd & q, Eigen::VectorXd & translation, Eigen::MatrixXd & orient){
    assert(("Invalid shape of given vector q", q.size()==dim));
    KDL::Frame fk_H;
    KDL::JntArray jnt(dim);
    eigen_to_kdl(q, jnt);
    fksolverpose->JntToCart(jnt, fk_H);
    kdl_to_eigen(fk_H.p, translation);
    for(int i=0; i<3; i++){
        for(int j =0; j<3; j++){
            orient(i, j) = fk_H.M.data[i*3+j];
        }
    }
}

uint RobotModel::getDim(){
    return dim;
}