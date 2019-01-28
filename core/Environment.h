#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "dart/dart.hpp"

namespace MSS
{
class Environment
{
public:
	Environment();
	Environment(int control_Hz,int simulation_Hz);

	void Step();

	const dart::simulation::WorldPtr& GetWorld(){return mWorld;}
private:
	dart::simulation::WorldPtr mWorld;
	int mControlHz,mSimulationHz;

	dart::dynamics::SkeletonPtr mCharacter;
	dart::dynamics::SkeletonPtr mGround;
};
};


#endif