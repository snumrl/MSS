#include "QP.h"
#include <iostream>
#include <sstream>
using namespace MSS;
void print_matrix(const char* name, const quadprogpp::Matrix<double>& A, int n=-1, int m=-1)
{
  std::ostringstream s;
  std::string t;
  if (n == -1)
    n = A.nrows();
  if (m == -1)
    m = A.ncols();
	
  s << name << ": " << std::endl;
  for (int i = 0; i < n; i++)
  {
    s << " ";
    for (int j = 0; j < m; j++)
      s << A[i][j] << ", ";
    s << std::endl;
  }
  t = s.str();
  t = t.substr(0, t.size() - 3); // To remove the trailing space, comma and newline
	
  std::cout << t << std::endl;
}
void print_vector(const char* name, const quadprogpp::Vector<double>& v, int n=-1)
{
  std::ostringstream s;
  std::string t;
  if (n == -1)
    n = v.size();
	
  s << name << ": " << std::endl << " ";
  for (int i = 0; i < n; i++)
  {
    s << v[i] << ", ";
  }
  t = s.str();
  t = t.substr(0, t.size() - 2); // To remove the trailing space and comma
	
  std::cout << t << std::endl;
}
QP::
QP(MSS::Character* character)
	:mCharacter(character),mW_tracking(1.0),mW_effort(0.1)
{
	mNumMuscles = mCharacter->GetMuscles().size();
	mNumDofs = mCharacter->GetSkeleton()->getNumDofs();
	mSolution.resize(mNumDofs+mNumMuscles);
	mQddDesired.resize(mNumDofs);

	mJt.resize(mNumMuscles);
	mA.resize(mNumMuscles);
	mP.resize(mNumMuscles);

	mM_minus_JtA.resize(mNumDofs,mNumDofs+mNumMuscles);
	mJtp_minus_c.resize(mNumDofs);	
	mJtA = Eigen::MatrixXd::Zero(mNumDofs,mNumMuscles);
	mJtp = Eigen::VectorXd::Zero(mNumDofs);
	mSolution.setZero();
	mQddDesired.setZero();

	mM_minus_JtA.setZero();
	mJtp_minus_c.setZero();	
	Initialize();
}
void
QP::
Minimize(const Eigen::VectorXd& qdd_desired)
{
	Update(qdd_desired);
	solve_quadprog(H,c,A,b,D,e,x);

	for(int i =0;i<mNumDofs+mNumMuscles;i++)
		mSolution[i] = x[i];
}
void
QP::
Initialize()
{
	H.resize(0.0,mNumDofs+mNumMuscles,mNumDofs+mNumMuscles);
	c.resize(0.0,mNumDofs+mNumMuscles);

	A.resize(0.0,mNumDofs+mNumMuscles,mNumDofs);
	b.resize(0.0,mNumDofs);

	D.resize(0.0,mNumDofs+mNumMuscles,mNumMuscles*2);
	e.resize(0.0,mNumMuscles*2);

	x.resize(0.0,mNumDofs+mNumMuscles);
	// x.resize(0.0,mNumMuscles);
	for(int i = 0;i<mNumDofs;i++)
		H[i][i] = sqrt(2.0*mW_tracking);

	for(int i = mNumDofs;i<mNumDofs+mNumMuscles;i++)
		H[i][i] = sqrt(2.0*mW_effort);

	for(int i = 0;i<mNumMuscles;i++)
		D[i+mNumDofs][i] = 1.0;
	for(int i = 0;i<mNumMuscles;i++)
		D[i+mNumDofs][i+mNumMuscles] = -1.0;

	for(int i = 0;i<mNumMuscles;i++)
		e[i] = 0.0;
	for(int i = 0;i<mNumMuscles;i++)
		e[i+mNumMuscles] = 1.0;
}
Eigen::MatrixXd
QP::
GetJtA()
{
	return mJtA;
}
Eigen::VectorXd
QP::
GetJtp_minus_c()
{
	return mJtp_minus_c;
}
void
QP::
Update(const Eigen::VectorXd& qdd_desired)
{
	mQddDesired = qdd_desired;

	auto& skel = mCharacter->GetSkeleton();
	auto& muscles = mCharacter->GetMuscles();

	mJtA.setZero();
	mJtp.setZero();
	for(int i =0;i<mNumMuscles;i++){
		mJt[i] = muscles[i]->GetJacobianTranspose();
		auto Ap_pair = muscles[i]->GetForceJacobianAndPassive();
		mA[i] = Ap_pair.first;
		mP[i] = Ap_pair.second;
		mJtA.block(0,i,mNumDofs,1) = mJt[i]*mA[i];
		mJtp += mJt[i]*mP[i];
	}

	mM_minus_JtA.setZero();
	mM_minus_JtA.block(0,0,mNumDofs,mNumDofs)= mCharacter->GetSkeleton()->getMassMatrix();

	mM_minus_JtA.block(0,mNumDofs,mNumDofs,mNumMuscles)= -mJtA;

	mJtp_minus_c = mJtp - (skel->getCoriolisAndGravityForces());

	//c
	for(int i = 0;i<mNumDofs;i++)
		c[i] = -2.0*mW_tracking*mQddDesired[i];

	//A
	for(int i=0;i<mNumDofs;i++)
		for(int j=0;j<mNumDofs+mNumMuscles;j++)
			A[j][i] = mM_minus_JtA(i,j);
	//b	
	for(int i=0;i<mNumDofs;i++)
		b[i] = -mJtp_minus_c[i];
}
