#ifndef __MSS_CHARACTER_H__
#define __MSS_CHARACTER_H__
#include "dart/dart.hpp"

namespace MSS
{
class BVH;
class Muscle;
class Character
{
public:
	Character();

	void LoadSkeleton(const std::string& path,bool create_bvh = true);
	void LoadMuscles(const std::string& path);
	void LoadBVH(const std::string& path);

	void SetPDParameters(double kp, double kv);
	Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& p_desired);

	Eigen::VectorXd GetTargetPositions(double t);
	
	const dart::dynamics::SkeletonPtr& GetSkeleton(){return mSkeleton;}
	const std::vector<Muscle*>& GetMuscles() {return mMuscles;}
	BVH* GetBVH(){return mBVH;}
public:
	dart::dynamics::SkeletonPtr mSkeleton;
	BVH* mBVH;
	std::vector<Muscle*> mMuscles;
	std::vector<dart::dynamics::BodyNode*> mEndEffectors;

	Eigen::VectorXd mKp, mKv;
};
};

#endif
