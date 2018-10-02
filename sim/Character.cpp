#include "Character.h"
#include "SkeletonBuilder.h"
#include <tinyxml.h>
using namespace dart::dynamics;

namespace MSS
{
Character::
Character(const dart::simulation::WorldPtr& world,const std::string& path)
	:mWorld(world)
{
	LoadSkeleton(path);
}
void
Character::
LoadSkeleton(const std::string& path)
{
	mSkeleton = MSS::SkeletonBuilder::BuildFromFile(path);
	//Load BVH map
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name");

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// name
		std::string name = body->Attribute("name");

		// bvh
		if(body->Attribute("bvh")!=nullptr){
			mBVHMap.insert(std::make_pair(name,body->Attribute("bvh")));
		}
	}
}
void
Character::
LoadMuscles(const std::string& path)
{
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");
	for(TiXmlElement* unit = muscledoc->FirstChildElement("Unit");unit!=nullptr;unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f0 = std::stod(unit->Attribute("f0"));
		double lm = std::stod(unit->Attribute("lm"));
		double lt = std::stod(unit->Attribute("lt"));
		double pa = std::stod(unit->Attribute("pen_angle"));
		mMuscles.push_back(new Muscle(name,f0,lm,lt,pa));
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
		{
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
			mMuscles.back()->AddAnchor(mSkeleton->getBodyNode(body),glob_pos);
		}
	}
	

}
void
Character::
LoadContactPoints(const std::string& path,double threshold,dart::dynamics::BodyNode* ground)
{
	std::vector<Eigen::Vector3d> joint_positions;
	for(int i=0;i<mSkeleton->getNumBodyNodes();i++)
	{
		auto p_joint = mSkeleton->getBodyNode(i)->getParentJoint();
		Eigen::Isometry3d T_body = mSkeleton->getBodyNode(i)->getTransform();
		Eigen::Isometry3d T_body_to_joint = p_joint->getTransformFromChildBodyNode();
		Eigen::Isometry3d T = T_body * T_body_to_joint;
		joint_positions.push_back(T.translation());
	}
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	double x,y,z;
	std::vector<Eigen::Vector3d> positions;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>x>>y>>z;
		Eigen::Vector3d vec;
		vec[0] = x;				
		vec[1] = y;				
		vec[2] = z;
		positions.push_back(vec);

		double min_val=1E6;
		int min_index=-1;
		for(int i=0;i<joint_positions.size();i++)
		{
			double distance = (vec-joint_positions[i]).norm();
			if(min_val>distance)
			{
				min_val = distance;
				min_index = i;
			}
		}
			
		mContactPoints.push_back(new ContactPoint(mWorld,mSkeleton->getBodyNode(min_index),ground,vec));
	}
	for(int i =0;i<positions.size();i++)
	{
		for(int j=i+1;j<positions.size();j++)
		{
			double distance = (positions[i]-positions[j]).norm();
			if(distance<threshold)
			{
				mContactPoints[i]->AddNeighbor(mContactPoints[j]);
				mContactPoints[j]->AddNeighbor(mContactPoints[i]);
			}
		}
	}
	ifs.close();
}
void
Character::
LoadMotionGraph(const std::string& path,const std::vector<int>& seq,double time_step)
{
	mMotionGraph = new MotionGraph(mSkeleton,mBVHMap,time_step);
	mMotionGraph->Parse(path);
	mMotionGraph->SetWalk(seq);
}

void
Character::
SetPDParameters(double kp, double kv)
{
	int dof = mSkeleton->getNumDofs();
	SetPDParameters(Eigen::VectorXd::Constant(dof, kp), Eigen::VectorXd::Constant(dof, kv));
}

void
Character::
SetPDParameters(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv)
{
	this->mKp = kp;
	this->mKv = kv;
	// this->mKp.segment<6>(0).setZero();
	// this->mKv.segment<6>(0).setZero();
}
#include <chrono>
Eigen::VectorXd
Character::
GetSPDForces(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired)
{
	// std::chrono::system_clock::time_point start,end;
	// start = std::chrono::system_clock::now();
	auto& skel = mSkeleton;
	Eigen::VectorXd q = skel->getPositions();
	Eigen::VectorXd dq = skel->getVelocities();
	double dt = skel->getTimeStep();
	Eigen::MatrixXd M_inv = (skel->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();
		
	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(skel->getPositionDifferences(qdqdt,p_desired));

	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq-v_desired);
	Eigen::VectorXd qddot = M_inv*(-skel->getCoriolisAndGravityForces()+
							p_diff+v_diff+skel->getConstraintForces());

	Eigen::VectorXd tau = p_diff + v_diff - dt*mKv.cwiseProduct(qddot);
	
	tau.head<6>().setZero();
	// end = std::chrono::system_clock::now();
	// std::chrono::duration<double> elapsed_seconds = end-start;
	// std::cout<<"GetSPDForces "<<" takes "<<elapsed_seconds.count()<<std::endl;
	return tau;
}
Eigen::VectorXd
Character::
GetSPDAccelerations(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired)
{
	auto& skel = mSkeleton;
	Eigen::VectorXd q = skel->getPositions();
	Eigen::VectorXd dq = skel->getVelocities();
	double dt = skel->getTimeStep();
	Eigen::MatrixXd M_inv = (skel->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();
		
	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(skel->getPositionDifferences(qdqdt,p_desired));

	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq-v_desired);
	Eigen::VectorXd cg = skel->getCoriolisAndGravityForces();
	Eigen::VectorXd qddot = M_inv*(-cg+	p_diff+v_diff+skel->getConstraintForces());

	Eigen::VectorXd tau = skel->getInvMassMatrix()*(p_diff + v_diff - dt*mKv.cwiseProduct(qddot)-cg);
	// tau.head<6>().setZero();
	for(int i =0;i<tau.rows();i++)
		tau[i] = dart::math::clip(tau[i],-1000.0,1000.0);
	return tau;
}
Eigen::VectorXd
Character::
GetTargetPositions(const Eigen::VectorXd& mode_lb)
{
	int dof = mSkeleton->getPositions().rows();
	
	Eigen::VectorXd p = mMotionGraph->GetMotion();

	for(int i =0;i<mode_lb.rows();i++){
		mMotionActions[i]->SetLB(mode_lb[i]);
		mMotionActions[i]->Set(p);
	}

	return p;
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetTargetPositionsAndVelocitiesFromBVH(const Eigen::VectorXd& mode_lb)
{
	Eigen::VectorXd p = GetTargetPositions(mode_lb);
	mMotionGraph->SaveState();
	mMotionGraph->Step();
	Eigen::VectorXd p1 = GetTargetPositions(mode_lb);
	mMotionGraph->LoadState();
	int dof = mSkeleton->getPositions().rows();

	Eigen::VectorXd target_position = p;
	Eigen::VectorXd target_velocity = (p1-p)/mMotionGraph->GetTimeStep();

	Eigen::VectorXd current_position = mSkeleton->getPositions();
	Eigen::VectorXd current_velocity = mSkeleton->getVelocities();
	
	mSkeleton->setPositions(target_position);
	mSkeleton->setVelocities(target_velocity);
	dynamic_cast<dart::dynamics::FreeJoint*>(mSkeleton->getRootJoint())->setLinearVelocity(target_velocity.segment<3>(3));
	mSkeleton->computeForwardKinematics(true,false,false);
	target_velocity = mSkeleton->getVelocities();

	mSkeleton->setPositions(current_position);
	mSkeleton->setVelocities(current_velocity);
	mSkeleton->computeForwardKinematics(true,false,false);
	return std::make_pair(target_position,target_velocity);
}
Eigen::VectorXd
Character::
GetIKTargetPositions(const Eigen::VectorXd& p,const Eigen::VectorXd& mode_lb)
{
	if(mode_lb.rows()==0)
		return p;
	Eigen::VectorXd p_IK = p;
	
	mIKAction->SetLB(mode_lb);
	mIKAction->Set(p_IK);

	return p_IK;
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetKpKv(double default_val,const Eigen::VectorXd& mode_lb)
{
	int dof = mSkeleton->getPositions().rows();
	Eigen::VectorXd kp = Eigen::VectorXd::Constant(dof,default_val);
	Eigen::VectorXd kv = Eigen::VectorXd::Constant(dof,default_val);
	
	for(int i =0;i<mode_lb.rows();i++){
		mKpActions[i]->SetLB(mode_lb[i]);
		mKpActions[i]->Set(kp);
	}
	for(int i =0;i<dof;i++)
		kv[i] = sqrt(2*kp[i]);
	return std::make_pair(kp,kv);
}
};