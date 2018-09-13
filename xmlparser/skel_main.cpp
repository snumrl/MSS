#include <tinyxml.h>
#include <Eigen/Core>
#include <map>
#include <vector>
#include <fstream>
#define LARGE_VALUE 1E6
#define REVOLUTE_JOINT_LIMIT 0.05
#define PROXIMAL_JOINT_LIMIT 1.0
namespace Eigen
{
	typedef Matrix<double,1,1> Vector1d;
};
struct UserConstant
{
	UserConstant(std::string parent,std::string joint_type,double mass,const Eigen::VectorXd& joint_lower_limit,const Eigen::VectorXd& joint_upper_limit,const Eigen::Vector3d& axis = Eigen::Vector3d::Zero(),std::string bvh_map="None",std::string obj_map="None")
		:mParent(parent),mJointType(joint_type),mMass(mass),mJointLowerLimit(joint_lower_limit),mJointUpperLimit(joint_upper_limit),mAxis(axis),mBVHMap(bvh_map),mOBJMap(obj_map)
	{

	}
	std::string mParent;
	std::string mJointType;
	double mMass;
	std::string mBVHMap;
	std::string mOBJMap;
	Eigen::VectorXd mJointLowerLimit;
	Eigen::VectorXd mJointUpperLimit;
	Eigen::Vector3d mAxis;
};
struct MayaConstant
{
	MayaConstant(std::string name, const Eigen::Vector3d& size,const Eigen::VectorXd& R,const Eigen::Vector3d& bt,const Eigen::Vector3d& jt)
		:mName(name),mSize(size),mBodyR(R),mBodyT(bt),mJointT(jt)
	{
		// std::cout<<name<<std::endl;
		// std::cout<<mSize.transpose()<<std::endl;
		// std::cout<<mBodyR.transpose()<<std::endl;
		// std::cout<<mBodyT.transpose()<<std::endl;
		// std::cout<<mJointT.transpose()<<std::endl<<std::endl;
	}
	std::string mName;
	Eigen::Vector3d mSize;
	Eigen::VectorXd mBodyR;
	Eigen::Vector3d mBodyT;
	Eigen::Vector3d mJointT;
};
std::string toString(const Eigen::Vector3d& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;

}
std::string toString(const Eigen::VectorXd& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;	
}
void ReadMayaConstant(std::vector<MayaConstant>& mc,std::string path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	double a1,a2,a3,a4,a5,a6,a7,a8,a9;
	std::string name;
	Eigen::Vector3d size;
	Eigen::VectorXd bodyR(9);
	Eigen::Vector3d bodyT;
	Eigen::Vector3d jointT;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;
		if(index=="n")
		{
			ss>>name;
		}
		else if(index=="s")
		{
			ss>>a1>>a2>>a3;
			size<<a1,a2,a3;
		}
		else if(index=="R")
		{
			ss>>a1>>a2>>a3>>a4>>a5>>a6>>a7>>a8>>a9;
			bodyR<<a1,a2,a3,a4,a5,a6,a7,a8,a9;
		}
		else if(index=="t")
		{
			ss>>a1>>a2>>a3;
			bodyT<<a1,a2,a3;
		}
		else if(index=="j")
		{
			ss>>a1>>a2>>a3;
			jointT<<a1,a2,a3;
			mc.push_back(MayaConstant(name,size,bodyR,bodyT,jointT));
		}
	}

	ifs.close();
}
#define DETAIL
int main(int argc,char** argv)
{
	std::map<std::string,UserConstant> ucs;
	std::vector<MayaConstant> mcs;
	ReadMayaConstant(mcs,argv[1]);
	ucs.insert(std::make_pair("Pelvis",UserConstant("None","FreeJoint",15.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Hips","Pelvis_mesh.obj")));
	//Upper body
	ucs.insert(std::make_pair("Spine",UserConstant("Pelvis","RevoluteJoint",10.0,Eigen::Vector1d(-0.4),Eigen::Vector1d(0.4),Eigen::Vector3d(0,1,0),"Character1_Spine","Spine_mesh.obj")));
	ucs.insert(std::make_pair("Torso",UserConstant("Spine","BallJoint",20.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Spine1","Torso_mesh.obj")));
	ucs.insert(std::make_pair("Neck",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-0.4),Eigen::Vector1d(0.4),Eigen::Vector3d(1,0,0),"Character1_Neck","Neck_mesh.obj")));
	ucs.insert(std::make_pair("Head",UserConstant("Neck","BallJoint",5.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_Head","Skull_mesh.obj")));
	ucs.insert(std::make_pair("ShoulderL",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_LeftShoulder","L_Shoulder_mesh.obj")));
	ucs.insert(std::make_pair("ArmL",UserConstant("ShoulderL","BallJoint",5.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftArm","L_Humerus_mesh.obj")));
	ucs.insert(std::make_pair("ForeArmL",UserConstant("ArmL","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_LeftForeArm","L_Radius_mesh.obj")));
	ucs.insert(std::make_pair("HandL",UserConstant("ForeArmL","BallJoint",1.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftHand","L_Hand_mesh.obj")));
	ucs.insert(std::make_pair("ShoulderR",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_RightShoulder","R_Shoulder_mesh.obj")));
	ucs.insert(std::make_pair("ArmR",UserConstant("ShoulderR","BallJoint",5.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightArm","R_Humerus_mesh.obj")));
	ucs.insert(std::make_pair("ForeArmR",UserConstant("ArmR","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_RightForeArm","R_Radius_mesh.obj")));
	ucs.insert(std::make_pair("HandR",UserConstant("ForeArmR","BallJoint",1.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightHand","R_Hand_mesh.obj")));
	//Lower body
	ucs.insert(std::make_pair("FemurL",UserConstant("Pelvis","BallJoint",5.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftUpLeg","L_Femur_mesh.obj")));
	ucs.insert(std::make_pair("TibiaL",UserConstant("FemurL","RevoluteJoint",2.0,Eigen::Vector1d(-0.3),Eigen::Vector1d(2.0),Eigen::Vector3d(1,0,0),"Character1_LeftLeg","L_Tibia_mesh.obj")));
	ucs.insert(std::make_pair("FemurR",UserConstant("Pelvis","BallJoint",5.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightUpLeg","R_Femur_mesh.obj")));
	ucs.insert(std::make_pair("TibiaR",UserConstant("FemurR","RevoluteJoint",2.0,Eigen::Vector1d(-0.3),Eigen::Vector1d(2.0),Eigen::Vector3d(1,0,0),"Character1_RightLeg","R_Tibia_mesh.obj")));

#ifndef DETAIL
	ucs.insert(std::make_pair("TalusL",UserConstant("TibiaL","BallJoint",1.5,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftFoot","L_Talus_merge_mesh.obj")));
	ucs.insert(std::make_pair("TalusR",UserConstant("TibiaR","BallJoint",1.5,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightFoot","R_Talus_merge_mesh.obj")));
#else
	//Left
	ucs.insert(std::make_pair("TalusL",UserConstant("TibiaL","BallJoint",0.3,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftFoot","L_Talus_mesh.obj")));
	ucs.insert(std::make_pair("NavicularL",UserConstant( "TalusL","WeldJoint",0.1,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"None","L_Navicular_mesh.obj")));
	ucs.insert(std::make_pair("MedialCuneiformL",UserConstant( "NavicularL","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_Medial_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal1L",UserConstant( "MedialCuneiformL","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Metatarsal_1_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx1L",UserConstant( "Metatarsal1L","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Proximal_Phalanx_1_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx1L",UserConstant( "ProximalPhalanx1L","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Distal_Phalanx_1_mesh.obj")));
	ucs.insert(std::make_pair("IntermediateCuneiformL",UserConstant( "NavicularL","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_Intermediate_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal2L",UserConstant( "IntermediateCuneiformL","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Metatarsal_2_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx2L",UserConstant( "Metatarsal2L","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Proximal_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx2L",UserConstant( "ProximalPhalanx2L","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Middle_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx2L",UserConstant( "MiddlePhalanx2L","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Distal_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("LateralCuneiformL",UserConstant( "NavicularL","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_Lateral_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal3L",UserConstant( "LateralCuneiformL","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Metatarsal_3_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx3L",UserConstant( "Metatarsal3L","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Proximal_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx3L",UserConstant( "ProximalPhalanx3L","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Middle_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx3L",UserConstant( "MiddlePhalanx3L","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Distal_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("CuboidL",UserConstant( "TalusL","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(0,0,1),"None","L_Cuboid_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal4L",UserConstant( "CuboidL","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Metatarsal_4_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx4L",UserConstant( "Metatarsal4L","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Proximal_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx4L",UserConstant( "ProximalPhalanx4L","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Middle_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx4L",UserConstant( "MiddlePhalanx4L","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Distal_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal5L",UserConstant( "CuboidL","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Metatarsal_5_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx5L",UserConstant( "Metatarsal5L","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Proximal_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx5L",UserConstant( "ProximalPhalanx5L","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Middle_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx5L",UserConstant( "MiddlePhalanx5L","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","L_F_Distal_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("CalcaneusL",UserConstant( "TalusL","WeldJoint",0.1,Eigen::Vector1d(-0.1),Eigen::Vector1d(0.1),Eigen::Vector3d(1,0,0),"None","L_Calcaneus_mesh.obj")));
	//Right
	ucs.insert(std::make_pair("TalusR",UserConstant("TibiaR","BallJoint",0.3,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightFoot","R_Talus_mesh.obj")));
	ucs.insert(std::make_pair("NavicularR",UserConstant( "TalusR","WeldJoint",0.1,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"None","R_Navicular_mesh.obj")));
	ucs.insert(std::make_pair("MedialCuneiformR",UserConstant( "NavicularR","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_Medial_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal1R",UserConstant( "MedialCuneiformR","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Metatarsal_1_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx1R",UserConstant( "Metatarsal1R","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Proximal_Phalanx_1_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx1R",UserConstant( "ProximalPhalanx1R","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Distal_Phalanx_1_mesh.obj")));
	ucs.insert(std::make_pair("IntermediateCuneiformR",UserConstant( "NavicularR","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_Intermediate_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal2R",UserConstant( "IntermediateCuneiformR","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Metatarsal_2_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx2R",UserConstant( "Metatarsal2R","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Proximal_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx2R",UserConstant( "ProximalPhalanx2R","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Middle_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx2R",UserConstant( "MiddlePhalanx2R","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Distal_Phalanx_2_mesh.obj")));
	ucs.insert(std::make_pair("LateralCuneiformR",UserConstant( "NavicularR","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_Lateral_Cuneiform_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal3R",UserConstant( "LateralCuneiformR","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Metatarsal_3_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx3R",UserConstant( "Metatarsal3R","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Proximal_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx3R",UserConstant( "ProximalPhalanx3R","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Middle_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx3R",UserConstant( "MiddlePhalanx3R","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Distal_Phalanx_3_mesh.obj")));
	ucs.insert(std::make_pair("CuboidR",UserConstant( "TalusR","WeldJoint",0.1,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(0,0,1),"None","R_Cuboid_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal4R",UserConstant( "CuboidR","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Metatarsal_4_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx4R",UserConstant( "Metatarsal4R","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Proximal_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx4R",UserConstant( "ProximalPhalanx4R","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Middle_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx4R",UserConstant( "MiddlePhalanx4R","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Distal_Phalanx_4_mesh.obj")));
	ucs.insert(std::make_pair("Metatarsal5R",UserConstant( "CuboidR","WeldJoint",0.07,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Metatarsal_5_mesh.obj")));
	ucs.insert(std::make_pair("ProximalPhalanx5R",UserConstant( "Metatarsal5R","WeldJoint",0.05,Eigen::Vector1d(-PROXIMAL_JOINT_LIMIT),Eigen::Vector1d(PROXIMAL_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Proximal_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("MiddlePhalanx5R",UserConstant( "ProximalPhalanx5R","WeldJoint",0.05,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Middle_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("DistalPhalanx5R",UserConstant( "MiddlePhalanx5R","WeldJoint",0.04,Eigen::Vector1d(-REVOLUTE_JOINT_LIMIT),Eigen::Vector1d(REVOLUTE_JOINT_LIMIT),Eigen::Vector3d(1,0,0),"None","R_F_Distal_Phalanx_5_mesh.obj")));
	ucs.insert(std::make_pair("CalcaneusR",UserConstant( "TalusR","WeldJoint",0.1,Eigen::Vector1d(-0.1),Eigen::Vector1d(0.1),Eigen::Vector3d(1,0,0),"None","R_Calcaneus_mesh.obj")));
#endif
	TiXmlDocument doc;
	TiXmlElement* skel_elem = new TiXmlElement("Skeleton");
	skel_elem->SetAttribute("name","Foot");
	doc.LinkEndChild(skel_elem);
	for(int i =0;i<mcs.size();i++)
	{
		TiXmlElement* joint_elem = new TiXmlElement("Joint");
		std::cout<<mcs[i].mName<<std::endl;
		if(ucs.find(mcs[i].mName)==ucs.end())
			continue;
		auto& uc = ucs.at(mcs[i].mName);
		// std::cout<<mcs[i].mName<<std::endl;

		joint_elem->SetAttribute("type",uc.mJointType);
		joint_elem->SetAttribute("name",mcs[i].mName);
		joint_elem->SetAttribute("parent_name",uc.mParent);
		joint_elem->SetAttribute("size",toString(mcs[i].mSize));
		joint_elem->SetAttribute("mass",std::to_string(uc.mMass));
		if(uc.mBVHMap!="None"){
			joint_elem->SetAttribute("bvh",uc.mBVHMap);
		}
		if(uc.mOBJMap!="None"){
			joint_elem->SetAttribute("obj",uc.mOBJMap);
		}
		if(uc.mAxis.norm()>1E-4)
		{
			joint_elem->SetAttribute("axis",toString(uc.mAxis));
		}
		TiXmlElement* body_position_elem = new TiXmlElement("BodyPosition");
		body_position_elem->SetAttribute("linear",toString(mcs[i].mBodyR));
		body_position_elem->SetAttribute("translation",toString(mcs[i].mBodyT));
		TiXmlElement* joint_position_elem = new TiXmlElement("JointPosition");
		joint_position_elem->SetAttribute("translation",toString(mcs[i].mJointT));
		if(uc.mJointLowerLimit.norm()<100.0)
		{
			joint_position_elem->SetAttribute("lower",toString(uc.mJointLowerLimit));
			joint_position_elem->SetAttribute("upper",toString(uc.mJointUpperLimit));	
		}
		
		joint_elem->LinkEndChild(body_position_elem);
		joint_elem->LinkEndChild(joint_position_elem);
		skel_elem->LinkEndChild( joint_elem );
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile(argv[2]);
	
	return 0;
}