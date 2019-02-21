#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "dart/dart.hpp"
namespace MSS
{
class Character;
class Environment
{
public:
	Environment(int control_Hz = 30,int simulation_Hz = 900);

	void Step();
	void Reset();
	bool IsEndOfEpisode();
	Eigen::VectorXd GetState();
	void SetAction(const Eigen::VectorXd& a);
	double GetReward();

	const dart::simulation::WorldPtr& GetWorld(){return mWorld;}
	Character* GetCharacter(){return mCharacter;}
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}
	int GetControlHz(){return mControlHz;}
	int GetSimulationHz(){return mSimulationHz;}
	int GetStateDofs();
	int GetActionDofs();
	int GetSystemDofs();
private:
	dart::simulation::WorldPtr mWorld;
	int mControlHz,mSimulationHz;

	Character* mCharacter;
	dart::dynamics::SkeletonPtr mGround;
	Eigen::VectorXd mAction;
};
};


#endif