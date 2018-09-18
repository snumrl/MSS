#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "Character.h"
#include "QP.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;

namespace MSS
{
class Character;
struct Tuple
{
	Eigen::VectorXd s;
	Eigen::VectorXd qdd_des;
	Eigen::VectorXd activation;
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
};
class Environment
{
public:
	Environment(int control_Hz=30,int simulation_Hz=900);
	
	Eigen::VectorXd ComputeActivationQP();	
	void Step(const Eigen::VectorXd& activation);
	void Reset(bool random=true);
	bool IsTerminalState();

	//For Deep RL
	Eigen::VectorXd GetState();
	double GetReward();
	Eigen::VectorXd GetAction(){return mAction;}
	void SetAction(const Eigen::VectorXd& a);
	int GetNumState(){return GetState().rows();};
	int GetNumAction(){return mAction.rows();};

	const dart::simulation::WorldPtr& GetWorld(){return mWorld;};
	Character* GetCharacter(){return mCharacter;};
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}	
	QP*	GetQP(){return mQP;}
	int GetControlHz(){return mControlHz;};
	int GetSimulationHz(){return mSimulationHz;};
	std::vector<Tuple>& GetTuples(){return mTuples;};
public:
	dart::simulation::WorldPtr mWorld;
	double mTimeElapsed;
	int mControlHz;
	int mSimulationHz;

	dart::dynamics::SkeletonPtr mGround;
	Character* mCharacter;
	
	double w_p,w_v,w_ee,w_com;
	Eigen::VectorXd mAction;

	QP*	mQP;
	std::pair<Eigen::VectorXd,Eigen::VectorXd> mTarget;
	Eigen::VectorXd mQddDesired;

	std::vector<Tuple> mTuples;
};

};


#endif