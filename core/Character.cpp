#include "Character.h"
#include "BVH.h"
#include "DARTHelper.h"
#include <tinyxml.h>
using namespace dart;
using namespace dart::dynamics;
using namespace MSS;
Character::
Character()
	:mSkeleton(nullptr),mBVH(nullptr)
{

}

void
Character::
LoadSkeleton(const std::string& path,bool create_bvh)
{
	mSkeleton = BuildFromFile(path);
	std::map<std::string,std::string> bvh_map;
	TiXmlDocument doc;
	doc.LoadFile(path);
	TiXmlElement *skel_elem = doc.FirstChildElement("Skeleton");

	for(TiXmlElement* node = skel_elem->FirstChildElement("Node");node != nullptr;node = node->NextSiblingElement("Node"))
	{
		if(node->Attribute("endeffector")!=nullptr)
		{
			if(node->Attribute("endeffector") == "True")
			{
				mEndEffectors.push_back(mSkeleton->getBodyNode(std::string(node->Attribute("name"))));
			}
		}
		TiXmlElement* joint_elem = node->FirstChildElement("Joint");
		if(joint_elem->Attribute("bvh")!=nullptr)
		{
			bvh_map.insert(std::make_pair(node->Attribute("name"),joint_elem->Attribute("bvh")));
		}
	}
	if(create_bvh)
		mBVH = new BVH(mSkeleton,bvh_map);
}

void
Character::
LoadBVH(const std::string& path)
{
	if(mBVH ==nullptr){
		std::cout<<"Initialize BVH class first"<<std::endl;
		return;
	}
	mBVH->Parse(path);
}

Eigen::VectorXd
Character::
GetTargetPositions(double t)
{
	return mBVH->GetMotion(t);
}