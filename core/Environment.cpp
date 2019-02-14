#include "Environment.h"
#include "DARTHelper.h"
#include "dart/collision/bullet/bullet.hpp"
using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace MSS;

Environment::
Environment()
	:Environment(30,900)
{

}

Environment::
Environment(int control_Hz,int simulation_Hz)
	:mControlHz(control_Hz),mSimulationHz(simulation_Hz),mWorld(std::make_shared<World>())
{
	mWorld->setGravity(Eigen::Vector3d(0,-9.8,0.0));
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
	mCharacter = BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/data/human.xml"));
	mGround = BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/data/ground.xml"));
	mWorld->addSkeleton(mCharacter);
	mWorld->addSkeleton(mGround);
}

void
Environment::
Step()
{
	mWorld->step();
}

