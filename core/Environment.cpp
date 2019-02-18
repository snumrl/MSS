#include "Environment.h"
#include "DARTHelper.h"
#include "Character.h"
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

	mCharacter = new Character();
	mCharacter->LoadSkeleton(std::string(MSS_ROOT_DIR)+std::string("/data/human.xml"),true);
	mCharacter->LoadMuscles(std::string(MSS_ROOT_DIR)+std::string("/data/muscle.xml"));
	mCharacter->LoadBVH(std::string(MSS_ROOT_DIR)+std::string("/data/motion/kick.bvh"));
	mGround = BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/data/ground.xml"));
	mWorld->addSkeleton(mCharacter->GetSkeleton());
	mWorld->addSkeleton(mGround);
}

void
Environment::
Step()
{
	mWorld->step();

	double t = mWorld->getTime();
	Eigen::VectorXd p_des = mCharacter->GetTargetPositions(t);
	int dof = mCharacter->GetSkeleton()->getNumDofs();
	
	mCharacter->GetSkeleton()->setPositions(p_des);
	mCharacter->GetSkeleton()->setVelocities(Eigen::VectorXd::Zero(dof));
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
}

void
Environment::
Reset()
{
	mWorld->reset();

	mCharacter->GetSkeleton()->clearConstraintImpulses();
	mCharacter->GetSkeleton()->clearInternalForces();
	mCharacter->GetSkeleton()->clearExternalForces();
	
	double t = mWorld->getTime();
	Eigen::VectorXd p_des = mCharacter->GetTargetPositions(t);
	int dof = mCharacter->GetSkeleton()->getNumDofs();

	mCharacter->GetSkeleton()->setPositions(p_des);
	mCharacter->GetSkeleton()->setVelocities(Eigen::VectorXd::Zero(dof));
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
}