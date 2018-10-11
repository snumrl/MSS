#ifndef __MSS_CHARACTER_H__
#define __MSS_CHARACTER_H__
#include "ContactPoint.h"
#include "MotionGraph.h"
#include "Action.h"
#include "Muscle.h"
#include "MuscleLBS.h"
#include "Functions.h"
#include "dart/dart.hpp"

#include <algorithm>
namespace MSS
{


class Character
{
public:
	Character(const dart::simulation::WorldPtr& world,const std::string& path);

	void LoadSkeleton(const std::string& path);
	void LoadMuscles(const std::string& path);
	void LoadContactPoints(const std::string& path,double threshold,dart::dynamics::BodyNode* ground);
	void LoadMotionGraph(const std::string& path,const std::vector<int>& seq,double time_step);
	
	void AddMotionAction(MotionAction* mode){mMotionActions.push_back(mode);};
	void AddKpAction(KpAction* mode){mKpActions.push_back(mode);};
	void SetIKAction(IKAction* mode){mIKAction = mode;};

	void AddInterestBody(const std::string& str){mInterestBodies.push_back(mSkeleton->getBodyNode(str));};
	void AddInterestBodies(const std::vector<std::string>& str_list){for(int i =0;i<str_list.size();i++) AddInterestBody(str_list[i]);};
	void AddEndEffector(const std::string& str){mEndEffectors.push_back(mSkeleton->getBodyNode(str));};

	void SetPDParameters(double kp, double kv);
	void SetPDParameters(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv);

	Eigen::VectorXd GetTargetPositions(const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetTargetPositionsAndVelocitiesFromBVH(const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
	Eigen::VectorXd GetIKTargetPositions(const Eigen::VectorXd& p,const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetKpKv(double default_val,const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
	Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired);
	Eigen::VectorXd GetSPDAccelerations(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired);
	
	const dart::dynamics::SkeletonPtr& GetSkeleton() {return mSkeleton;};
	const std::vector<MuscleLBS*>& GetMuscles() {return mMuscles;};
	const std::vector<ContactPoint*>& GetContactPoints() {return mContactPoints;};
	MotionGraph* GetMotionGraph(){return mMotionGraph;};
	const std::vector<dart::dynamics::BodyNode*>& GetInterestBodies() {return mInterestBodies;};
	const std::vector<dart::dynamics::BodyNode*>& GetEndEffectors() {return mEndEffectors;};
	const std::vector<MotionAction*>& GetMotionActions(){return mMotionActions;};
	const std::vector<KpAction*>& GetKpActions(){return mKpActions;};
	IKAction* GetIKAction(){return mIKAction;};
	const std::map<std::string,std::string>& GetBVHMap(){return mBVHMap;};
	
public:
	dart::simulation::WorldPtr mWorld;
	dart::dynamics::SkeletonPtr mSkeleton;
	std::vector<MuscleLBS*> mMuscles;
	std::map<std::string,std::string> mBVHMap;
	MotionGraph* mMotionGraph;
	Eigen::VectorXd mKp, mKv;
	std::vector<MotionAction*> mMotionActions;
	std::vector<KpAction*> mKpActions;
	IKAction* mIKAction;
	std::vector<ContactPoint*> mContactPoints;
	std::vector<dart::dynamics::BodyNode*> mEndEffectors;
	std::vector<dart::dynamics::BodyNode*> mInterestBodies;
};
};
#endif