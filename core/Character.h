#ifndef __MSS_CHARACTER_H__
#define __MSS_CHARACTER_H__
#include "dart/dart.hpp"

namespace MSS
{
class BVH;
class Character
{
public:
	Character();

	void LoadSkeleton(const std::string& path,bool create_bvh = true);
	void LoadBVH(const std::string& path);

	Eigen::VectorXd GetTargetPositions(double t);

	BVH* GetBVH(){return mBVH;}
	const dart::dynamics::SkeletonPtr& GetSkeleton(){return mSkeleton;}
public:
	dart::dynamics::SkeletonPtr mSkeleton;
	BVH* mBVH;
	

	std::vector<dart::dynamics::BodyNode*> mEndEffectors;
};
};

#endif
