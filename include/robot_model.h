#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <math.h>


#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>
#include <functional>

#include <fstream>

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &mat, float tolerance = 1e-4) // choose appropriately
{
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = 1.0 / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = 0.0;
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

Eigen::VectorXd logmapSO3(Eigen::MatrixXd & orient){
	Eigen::VectorXd w = Eigen::VectorXd::Zero(3);
	float cos_th = 1/2.0*(orient.trace() - 1);
	float sin_th  =  1/2.0*sqrtf( (3-orient.trace())*(1+orient.trace()) );
	float theta = std::atan2(sin_th, cos_th);
	w << orient(2,1)-orient(1,2), orient(0,2)-orient(2,0), orient(1,0)-orient(0,1);
	return w*theta/(2*sin_th);
}

template<typename T>
void eigen_to_kdl(Eigen::VectorXd & q, T & t);

template<typename T>
void kdl_to_eigen(T & t, Eigen::VectorXd & q);


class RobotModel{
	KDL::Vector gravity;
	std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolverpose;
	std::shared_ptr<KDL::ChainJntToJacSolver> jacsolver;
	std::shared_ptr<KDL::ChainJntToJacDotSolver> djacsolver;

    KDL::Tree tree;
	KDL::Chain chain;

	int dim;

public:
	RobotModel(std::string urdf_filename, std::string base_link, std::string tool_link);
    void calcJacobian(Eigen::VectorXd & q, Eigen::MatrixXd & jac);
    void fk(Eigen::VectorXd & q, Eigen::VectorXd & pose, Eigen::MatrixXd & orient);
	uint getDim();
};