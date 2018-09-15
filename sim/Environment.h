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
class Environment
{
public:
	Environment(int control_Hz=30,int simulation_Hz=900);
	
	void Step();
	void Reset(bool random=true);
	bool IsTerminalState();

	//For Deep RL
	Eigen::VectorXd GetState();
	double GetReward();
	Eigen::VectorXd GetAction(){return mAction;}
	void SetAction(const Eigen::VectorXd& a);
	int GetNumState(){return GetState().rows();};
	int GetNumAction(){return mAction.rows();};

	//For Muscle Regressor
	void LoadMuscleNN(const std::string& path);
	Eigen::VectorXd GetActivationFromNN();

	const dart::simulation::WorldPtr& GetWorld(){return mWorld;};
	Character* GetCharacter(){return mCharacter;};
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}	
	QP*	GetQP(){return mQP;}
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

	//For Muscle Regressor
	bool mIsNNLoaded;
	p::object mm,mns,sys_module,nn_module;
};
};


#endif