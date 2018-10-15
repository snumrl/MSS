#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "Character.h"
#include "QP.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "MuscleLimitConstraint.h"

namespace p = boost::python;
namespace np = boost::python::numpy;

namespace MSS
{
class Character;
struct Tuple
{
	Eigen::VectorXd tau;
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
	Eigen::VectorXd tau_des;
	
};
class Environment
{
public:
	Environment(int control_Hz=30,int simulation_Hz=900);
	void Step(const Eigen::VectorXd& activation);

	void Reset(bool random=true);
	Eigen::VectorXd GetMuscleTorques();
	bool IsTerminalState();

	//For Deep RL
	Eigen::VectorXd GetState();
	double GetReward();
	Eigen::VectorXd GetAction(){return mAction;}
	void SetAction(const Eigen::VectorXd& a);
	void SetAlpha(double a){mAlpha = a;};

	int GetNumState(){return GetState().rows();};
	int GetNumAction(){return mAction.rows();};
	int GetNumTotalRelatedDofs(){return mNumTotalRelatedDofs;};
	const dart::simulation::WorldPtr& GetWorld(){return mWorld;};
	Character* GetCharacter(){return mCharacter;};
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}	
	QP*	GetQP(){return mQP;}
	int GetControlHz(){return mControlHz;};
	int GetSimulationHz(){return mSimulationHz;};
	std::vector<Tuple>& GetTuples(){return mTuples;};
	Eigen::VectorXd GetDesiredTorques();
	// const Eigen::VectorXd& GetDesiredAcceleration(){mQddDesired = mCharacter->GetSPDAccelerations(mTarget.first,mTarget.second);return mQddDesired;};
	double GetElapsedTime(){return mTimeElapsed;}
	std::shared_ptr<dart::constraint::WeldJointConstraint>& GetWeldConstraint(){return mWeldConstraint;};
public:
	dart::simulation::WorldPtr mWorld;
	double mTimeElapsed;
	int mControlHz;
	int mSimulationHz;
	int mNumTotalRelatedDofs;
	int mSimCount;
	int mRandomSampleIndex;
	double mAlpha;
	bool mIsFirstAction;

	dart::dynamics::SkeletonPtr mGround;
	Character* mCharacter;
	
	double w_p,w_v,w_ee,w_com;
	Eigen::VectorXd mAction;

	QP*	mQP;
	std::pair<Eigen::VectorXd,Eigen::VectorXd> mTarget;
	Eigen::VectorXd mTorqueDesired;

	std::vector<Tuple> mTuples;
	Tuple mTempTuple;

	Eigen::VectorXd mMeasurePose;
	std::shared_ptr<dart::constraint::WeldJointConstraint> mWeldConstraint;
};

};


#endif