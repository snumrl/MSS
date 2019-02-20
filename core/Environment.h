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
	
	const dart::simulation::WorldPtr& GetWorld(){return mWorld;}
	Character* GetCharacter(){return mCharacter;}
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}
	int GetControlHz(){return mControlHz;}
	int GetSimulationHz(){return mSimulationHz;}
private:
	dart::simulation::WorldPtr mWorld;
	int mControlHz,mSimulationHz;

	Character* mCharacter;
	dart::dynamics::SkeletonPtr mGround;
};
};


#endif