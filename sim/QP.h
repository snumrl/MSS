#ifndef __QP_H__
#define __QP_H__
#include "Character.h"
#include "quadprog_optimized/QuadProg++.hh"
#include "dart/dart.hpp"
#include "dart/simulation/simulation.hpp"

namespace MSS
{
class QP
{
public:
	QP(MSS::Character* Character);
	void Minimize(const Eigen::VectorXd& qdd_desired);
	Eigen::VectorXd GetSolution(){return mSolution;}
	void Update(const Eigen::VectorXd& qdd_desired);
	Eigen::MatrixXd GetJtA();
	Eigen::VectorXd GetJtp_minus_c();
private:
	void Initialize();
	MSS::Character* mCharacter;
	int mNumDofs,mNumMuscles;
	std::vector<Eigen::MatrixXd> mJt,mA;
	std::vector<Eigen::VectorXd> mP;

	Eigen::MatrixXd mJtA;
	Eigen::VectorXd mJtp;
	Eigen::MatrixXd mM_minus_JtA;
	Eigen::VectorXd mJtp_minus_c;

	double mW_effort,mW_tracking;

	Eigen::VectorXd mSolution;
	Eigen::VectorXd mQddDesired;
//Objective
	quadprogpp::Matrix<double> H;
	quadprogpp::Vector<double> c;
//Equality
	quadprogpp::Matrix<double> A;
	quadprogpp::Vector<double> b;
//Inequality
	quadprogpp::Matrix<double> D;
	quadprogpp::Vector<double> e;
//solution
	quadprogpp::Vector<double> x;
};
}
#endif