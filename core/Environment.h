#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "dart/dart.hpp"
namespace MSS
{
class Character;
class Environment
{
public:
	Environment();
	Environment(int control_Hz,int simulation_Hz);

	void Step();
	void Reset();
	
	const dart::simulation::WorldPtr& GetWorld(){return mWorld;}
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