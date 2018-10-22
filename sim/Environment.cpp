#include "Environment.h"
#include "SkeletonBuilder.h"
#include <initializer_list>
#include <chrono>
namespace MSS
{

Environment::
Environment(int control_Hz,int simulation_Hz)
	:mControlHz(control_Hz),mSimulationHz(simulation_Hz),mWorld(std::make_shared<dart::simulation::World>()),w_p(0.65),w_v(0.1),w_ee(0.15),w_com(0.1),mAlpha(1.0),mRandomSampleIndex(-1),mSimCount(-1),mIsFirstAction(true)
{
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0));
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	// this->mWorld->getConstraintSolver()->setLCPSolver(dart::common::make_unique<dart::constraint::PGSLCPSolver>(mWorld->getTimeStep()));
	mGround = MSS::SkeletonBuilder::BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/character/ground.xml"));
	mWorld->addSkeleton(mGround);
	mCharacter = new Character(mWorld,std::string(MSS_ROOT_DIR)+std::string("/character/full_body.xml"));
	mWorld->addSkeleton(mCharacter->GetSkeleton());

	mCharacter->LoadMuscles(std::string(MSS_ROOT_DIR)+std::string("/character/muscle.xml"));
	mNumTotalRelatedDofs = 0;
	for(auto m : mCharacter->GetMuscles()){
		m->Update(0.01);
		mNumTotalRelatedDofs += m->GetNumRelatedDofs();
	}

	std::cout<<"# muscles : "<<mCharacter->GetMuscles().size()<<std::endl;
	std::cout<<"# muscles dofs : "<<mNumTotalRelatedDofs<<std::endl;
	//Hard-coded parameter
	// mCharacter->LoadContactPoints(std::string(MSS_ROOT_DIR)+std::string("/character/txt/contact.txt"),0.035,mGround->getBodyNode(0));
	
	mCharacter->LoadMotionGraph(std::string(MSS_ROOT_DIR)+std::string("/motion/simple.graph"),std::vector<int>{0,0,0,0,0},1.0/(double)mControlHz);
	mCharacter->AddInterestBodies(std::vector<std::string> {"FemurR","TibiaR","TalusR"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"FemurL","TibiaL","TalusL"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"Spine","Torso","Neck","Head"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"ShoulderR","ArmR","ForeArmR","HandR"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"ShoulderL","ArmL","ForeArmL","HandL"});
	mCharacter->AddEndEffector("TalusR");
	mCharacter->AddEndEffector("TalusL");
	mCharacter->AddEndEffector("HandR");
	mCharacter->AddEndEffector("HandL");
	mCharacter->AddEndEffector("Head");

	Eigen::VectorXd zeros = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs());
	
	double min_kp=10.0,max_kp=500.0;
	{
		std::string name = "Pelvis";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<6;i++)
		{
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#0,#1,#2
	{
		std::string name = "FemurR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#3,#4,#5
	{
		std::string name = "FemurL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#6
	{
		std::string name = "TibiaR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#7
	{
		std::string name = "TibiaL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#8,#9,#10
	{
		std::string name = "TalusR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#11,#12,#13
	{
		std::string name = "TalusL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#14
	{
		std::string name = "Spine";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#15,#16,#17
	{
		std::string name = "Torso";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#18
	{
		std::string name = "Neck";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#19,#20,#21
	{
		std::string name = "Head";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#22
	{
		std::string name = "ShoulderR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#23
	{
		std::string name = "ShoulderL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#24,#25,#26
	{
		std::string name = "ArmR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#27,#28,#29
	{
		std::string name = "ArmL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#30
	{
		std::string name = "ForeArmR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#31
	{
		std::string name = "ForeArmL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
		p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
		p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
		kp_lower[index] = min_kp;
		kp_upper[index] = max_kp;
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#32,#33,#34
	{
		std::string name = "HandR";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#35,#36,#37
	{
		std::string name = "HandL";
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		for(int i =0;i<3;i++)
		{
			Eigen::VectorXd p_lower = zeros;
			Eigen::VectorXd p_upper = zeros;
			int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(i)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
			mCharacter->AddMotionAction(new MotionAction(name+std::to_string(i),p_upper,p_lower));
		}
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#38
	{
		std::string name = "FistR";
		std::vector<std::string> related_bodies{"FootThumbR","FootPinkyR"};
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		for(int i =0;i<2;i++)
		{
			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
		}
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	//#39
	{
		std::string name = "FistL";
		std::vector<std::string> related_bodies{"FootThumbL","FootPinkyL"};
		Eigen::VectorXd kp_upper = zeros;
		Eigen::VectorXd kp_lower = zeros;
		Eigen::VectorXd p_lower = zeros;
		Eigen::VectorXd p_upper = zeros;
		for(int i =0;i<2;i++)
		{
			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getPositionLowerLimit();
			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[i])->getParentJoint()->getDof(0)->getPositionUpperLimit();
			kp_lower[index] = min_kp;
			kp_upper[index] = max_kp;
		}
		mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
		mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	}
	// //#14
	// {
	// 	std::string name = "FistThumbR";
	// 	std::vector<std::vector<std::string>> related_bodies{
	// 		{"DistalPhalanx1R","DistalPhalanx2R","DistalPhalanx3R","DistalPhalanx4R","DistalPhalanx5R"},
	// 		{"MiddlePhalanx1R","MiddlePhalanx2R","MiddlePhalanx3R","MiddlePhalanx4R","MiddlePhalanx5R"},
	// 		{"ProximalPhalanx1R","ProximalPhalanx2R","ProximalPhalanx3R","ProximalPhalanx4R","ProximalPhalanx5R"}};
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	for(int i =0;i<5;i++)
	// 	{
	// 		double related_ratio = 1.0-1.0/4.0*i;
	// 		for(int j =0;j<3;j++)
	// 		{
	// 			if(i==0&&j==1)
	// 				continue;
				
	// 			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionLowerLimit()*related_ratio;
	// 			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionUpperLimit()*related_ratio;
	// 			kp_lower[index] = min_kp*related_ratio;
	// 			kp_upper[index] = max_kp*related_ratio;
	// 		}
				
			
	// 	}
	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	// //#15
	// {
	// 	std::string name = "FistPinkyR";
	// 	std::vector<std::vector<std::string>> related_bodies{
	// 		{"DistalPhalanx5R","DistalPhalanx4R","DistalPhalanx3R","DistalPhalanx2R","DistalPhalanx1R"},
	// 		{"MiddlePhalanx5R","MiddlePhalanx4R","MiddlePhalanx3R","MiddlePhalanx2R","MiddlePhalanx1R"},
	// 		{"ProximalPhalanx5R","ProximalPhalanx4R","ProximalPhalanx3R","ProximalPhalanx2R","ProximalPhalanx1R"}};
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	for(int i =0;i<5;i++)
	// 	{
	// 		double related_ratio = 1.0-1.0/4.0*i;
	// 		for(int j =0;j<3;j++)
	// 		{
	// 			if(i==4&&j==1)
	// 				continue;
				
	// 			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionLowerLimit()*related_ratio;
	// 			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionUpperLimit()*related_ratio;
	// 			kp_lower[index] = min_kp*related_ratio;
	// 			kp_upper[index] = max_kp*related_ratio;
	// 		}
				
			
	// 	}

	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	// //#16
	// {
	// 	std::string name = "FistThumbL";
	// 	std::vector<std::vector<std::string>> related_bodies{
	// 		{"DistalPhalanx1L","DistalPhalanx2L","DistalPhalanx3L","DistalPhalanx4L","DistalPhalanx5L"},
	// 		{"MiddlePhalanx1L","MiddlePhalanx2L","MiddlePhalanx3L","MiddlePhalanx4L","MiddlePhalanx5L"},
	// 		{"ProximalPhalanx1L","ProximalPhalanx2L","ProximalPhalanx3L","ProximalPhalanx4L","ProximalPhalanx5L"}};
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	for(int i =0;i<5;i++)
	// 	{
	// 		double related_ratio = 1.0-1.0/4.0*i;
	// 		for(int j =0;j<3;j++)
	// 		{
	// 			if(i==0&&j==1)
	// 				continue;
				
	// 			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionLowerLimit()*related_ratio;
	// 			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionUpperLimit()*related_ratio;
	// 			kp_lower[index] = min_kp*related_ratio;
	// 			kp_upper[index] = max_kp*related_ratio;
	// 		}
				
			
	// 	}
	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	// //#17
	// {
	// 	std::string name = "FistPinkyL";
	// 	std::vector<std::vector<std::string>> related_bodies{
	// 		{"DistalPhalanx5L","DistalPhalanx4L","DistalPhalanx3L","DistalPhalanx2L","DistalPhalanx1L"},
	// 		{"MiddlePhalanx5L","MiddlePhalanx4L","MiddlePhalanx3L","MiddlePhalanx2L","MiddlePhalanx1L"},
	// 		{"ProximalPhalanx5L","ProximalPhalanx4L","ProximalPhalanx3L","ProximalPhalanx2L","ProximalPhalanx1L"}};
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	for(int i =0;i<5;i++)
	// 	{
	// 		double related_ratio = 1.0-1.0/4.0*i;
	// 		for(int j =0;j<3;j++)
	// 		{
	// 			if(i==4&&j==1)
	// 				continue;
				
	// 			int index = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 			p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionLowerLimit()*related_ratio;
	// 			p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(related_bodies[j][i])->getParentJoint()->getDof(0)->getPositionUpperLimit()*related_ratio;
	// 			kp_lower[index] = min_kp*related_ratio;
	// 			kp_upper[index] = max_kp*related_ratio;
	// 		}
				
			
	// 	}

	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	// //#18
	// {
	// 	std::string name = "CalcaneusR";
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 	p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
	// 	p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
	// 	kp_lower[index] = min_kp;
	// 	kp_upper[index] = max_kp;
	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	// //#19
	// {
	// 	std::string name = "CalcaneusL";
	// 	Eigen::VectorXd kp_upper = zeros;
	// 	Eigen::VectorXd kp_lower = zeros;
	// 	Eigen::VectorXd p_lower = zeros;
	// 	Eigen::VectorXd p_upper = zeros;
	// 	int index = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getIndexInSkeleton();
	// 	p_lower[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionLowerLimit();
	// 	p_upper[index] = mCharacter->GetSkeleton()->getBodyNode(name)->getParentJoint()->getDof(0)->getPositionUpperLimit();
	// 	kp_lower[index] = min_kp;
	// 	kp_upper[index] = max_kp;
	// 	mCharacter->AddMotionAction(new MotionAction(name,p_upper,p_lower));
	// 	mCharacter->AddKpAction(new KpAction(name+"_kp",kp_upper,kp_lower));
	// }
	{
		std::vector<std::string> target_name = {"TalusL","TalusR"};
		// std::vector<std::string> target_name = {};
		std::vector<dart::dynamics::BodyNode*> target_body;
		Eigen::VectorXd p_upper(target_name.size()*3);
		Eigen::VectorXd p_lower(target_name.size()*3);
		for(int i =0;i<target_name.size();i++){
			target_body.push_back(mCharacter->GetSkeleton()->getBodyNode(target_name[i]));
			p_upper.segment<3>(i*3) = Eigen::Vector3d(0.1,0.1,0.1);
			p_lower.segment<3>(i*3) = Eigen::Vector3d(-0.1,-0.1,-0.1);
		}
		
		

		mCharacter->SetIKAction(new IKAction("WholeBodyIK",mCharacter->GetSkeleton(),target_body,p_upper,p_lower));
	}

	// mAction = Eigen::VectorXd::Zero(mCharacter->GetMotionActions().size()+mCharacter->GetIKAction()->GetDof()+mCharacter->GetKpActions().size());
	mAction = Eigen::VectorXd::Zero(mCharacter->GetMotionActions().size());

	// mAction = Eigen::VectorXd::Zero(mCharacter->GetMuscles().size());
	auto kpkv = mCharacter->GetKpKv(300.0);
	mCharacter->SetPDParameters(kpkv.first,kpkv.second);
	mQP = new QP(mCharacter);
	mMeasurePose = mCharacter->GetSkeleton()->getPositions();
	mMeasurePose.setZero();
	// mMeasurePose[0] = -1.57;
	// mMeasurePose[4] = -0.85;
	// mMeasurePose[4] = -0.85;
	// mMeasurePose[4] = 0.0;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("FemurL")->getDof(0)->getIndexInSkeleton()] = -1.57;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("TibiaL")->getDof(0)->getIndexInSkeleton()] = 0.5;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("TibiaL")->getDof(0)->getIndexInSkeleton()] = 1.57;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("ArmL")->getDof(2)->getIndexInSkeleton()] = -1.57;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("ForeArmL")->getDof(0)->getIndexInSkeleton()] = -2.5;
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("TibiaL")->getDof(0)->getIndexInSkeleton()] = 0.5;
	for(int i=0;i<mCharacter->GetMuscles().size();i++)
	{
		if(mCharacter->GetMuscles()[i]->l_mt_max>0.0)
		{
			mCharacter->GetMuscles()[i]->Update(mWorld->getTimeStep());
			mMuscleLimitConstraints.push_back(std::make_shared<MuscleLimitConstraint>(mCharacter->GetMuscles()[i]));
			mWorld->getConstraintSolver()->addConstraint(mMuscleLimitConstraints.back());	
		}
		
	}
	Reset(false);
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("Pelvis")));
	mWeldConstraint = std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("TalusL"));
	// mWorld->getConstraintSolver()->addConstraint(mWeldConstraint);
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("HandL")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("HandR")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("Head")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("TalusR")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("FemurL")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("TalusL")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("ShoulderL")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("Head")));
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode("HandL"),mCharacter->GetSkeleton()->getBodyNode("ForeArmL")));
	// mWeldConstraint->setErrorReductionParameter(0.002);
	// mWeldConstraint->setConstraintForceMixing(1E-2);
	// for(int i =0;i<mCharacter->GetSkeleton()->getNumJoints();i++)
	// {
	// 	if(mCharacter->GetSkeleton()->getJoint(i)->getName()!="TibiaL" || mCharacter->GetSkeleton()->getJoint(i)->getName()!="FemurL")
	// 	{
	// 		std::size_t dof = mCharacter->GetSkeleton()->getJoint(i)->getNumDofs();
	// 		for(int j=0;j<dof;j++)
	// 		{
	// 			mCharacter->GetSkeleton()->getJoint(i)->setDampingCoefficient(j,1000.0);
	// 		}	
	// 	}
	// }

	
}
void
Environment::
Step(const Eigen::VectorXd& activation)
{
	// For Kinematic Moves
	// int count_ = 0;
	// for(auto muscle : mCharacter->GetMuscles())
	// {
	// 	muscle->activation = activation[count_++];
	// 	muscle->Update(mWorld->getTimeStep());
	// }
	// mCharacter->GetSkeleton()->setPositions(mTarget.first);
	// mCharacter->GetSkeleton()->setVelocities(mTarget.second);
	// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	// return;
	
	//For Muscle Actuator
	// int count = 0;
	// for(auto muscle : mCharacter->GetMuscles())
	// {
	// 	muscle->activation = activation[count++];
	// 	if(mSimCount%4==0)
	// 		muscle->Update(mWorld->getTimeStep());
	// 	// muscle->ApplyForceToBody();
	// }

	for(int j=0;j<mMuscleLimitConstraints.size();j++)
	{
		auto muscle = mMuscleLimitConstraints[j]->GetMuscle();
		muscle->Update(0.01);
	}
	// Eigen::Isometry3d T_weld = mWeldConstraint->getBodyNode1()->getTransform();
	// Eigen::Isometry3d T_target = mWeldConstraint->getRelativeTransform();
	// Eigen::Vector3d p = (T_target.translation()-T_weld.translation());
	// Eigen::Vector3d v = mWeldConstraint->getBodyNode1()->getCOMLinearVelocity();
	// double kp,kv;
	// kp = 5000.0;
	// kv = 2*sqrt(kp);
	// Eigen::Vector3d weld_force = kp*p - kv*v;
	// mWeldConstraint->getBodyNode1()->addExtForce(weld_force);

	// Eigen::VectorXd muscle_force = mCharacter->GetSkeleton()->getExternalForces();
	// mCharacter->GetSkeleton()->clearExternalForces();
	mTorqueDesired = mCharacter->GetSPDForces(mTarget.first,mTarget.second);
	mCharacter->GetSkeleton()->setForces(mTorqueDesired);	
	// Eigen::VectorXd error = mTorqueDesired - muscle_force;

	if(mRandomSampleIndex==mSimCount)
	{
		Tuple tp;
		tp.tau = mTempTuple.tau;
		tp.tau_des = mTorqueDesired.tail(mTorqueDesired.rows()-6);
		tp.A = mTempTuple.A;
		tp.b = mTempTuple.b;

		mTuples.push_back(tp);
	}
	
	//For Joint Torque
	

	mWorld->step();
	// Eigen::VectorXd p = mCharacter->GetSkeleton()->getPositions();
	// Eigen::VectorXd v = mCharacter->GetSkeleton()->getVelocities();
	// p.segment<3>(3).setZero();
	// v.head<6>(0).setZero();
	// mCharacter->GetSkeleton()->setPositions(p);
	// mCharacter->GetSkeleton()->setVelocities(v);
	mSimCount++;
}
Eigen::VectorXd
Environment::
GetDesiredTorques()
{
	mTorqueDesired = mCharacter->GetSPDForces(mTarget.first,mTarget.second);
	// mTorqueDesired = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs());
	return mTorqueDesired.tail(mTorqueDesired.rows()-6);
}
Eigen::VectorXd
Environment::
GetMuscleTorques()
{
	int index = 0;
	Eigen::VectorXd JtA = Eigen::VectorXd::Zero(mNumTotalRelatedDofs);
	for(auto muscle : mCharacter->GetMuscles())
	{
		muscle->Update(mWorld->getTimeStep());
		Eigen::VectorXd JtA_i = muscle->GetRelatedJtA();
		JtA.segment(index,JtA_i.rows()) = JtA_i;
		index+=JtA_i.rows();
	}
	mTempTuple.tau = JtA;
	return JtA;
}
void
Environment::
Reset(bool random)
{
	mWorld->reset();

	mCharacter->GetSkeleton()->clearConstraintImpulses();
	mCharacter->GetSkeleton()->clearInternalForces();
	mCharacter->GetSkeleton()->clearExternalForces();
	
	if(random)
		mTimeElapsed = dart::math::random(0.0,mCharacter->GetMotionGraph()->GetMaxTimeOfFirstSeq());
	else
		mTimeElapsed = 0.0;
	mCharacter->GetMotionGraph()->Reset(mTimeElapsed);
	mAction.setZero();
	
	mIsFirstAction = false;
	mTarget = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();

	// mCharacter->GetSkeleton()->setPositions(target.first);
	// mCharacter->GetSkeleton()->setVelocities(target.second);
	// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);

	// // Eigen::VectorXd sin_pose = mMeasurePose;
	// // sin_pose.setZero();
	// // sin_pose[mCharacter->GetSkeleton()->getJoint("TibiaL")->getDof(0)->getIndexInSkeleton()] = 0.8*sin(mTimeElapsed*3.0)-0.5;
	// mTarget.first = mMeasurePose;

	// mTarget.second = mMeasurePose;
	// mTarget.second.setZero();
	mCharacter->GetSkeleton()->setPositions(mTarget.first);
	mCharacter->GetSkeleton()->setVelocities(mTarget.second);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	// return;
	//Muscle Inequality handling
	Eigen::VectorXd cur_pos = mTarget.first;
	bool collision_happy = false;

	for(int i =0;i<1000;i++)
	{
		std::vector<bool> active;
		std::vector<double> l_mt;
		active.resize(mMuscleLimitConstraints.size(),false);
		l_mt.resize(mMuscleLimitConstraints.size(),-1.0);
		//Check Inequality5
		for(int j=0;j<mMuscleLimitConstraints.size();j++)
		{
			auto muscle = mMuscleLimitConstraints[j]->GetMuscle();

			muscle->Update(0.01);
			if(muscle->l_mt > muscle->l_mt_max){
				// std::cout<<muscle->name<<muscle->l_mt_max - muscle->l_mt<<std::endl;
				l_mt[j] = muscle->l_mt;
				active[j] = true;
			}
		}

		// std::cout<<std::all_of(active.begin(), active.end(), [](bool i){return i==false;})<<std::endl;
		// std::cout<<collision_happy<<std::endl;
		//Compute Gradient for active inequality
		if( std::all_of(active.begin(), active.end(), [](bool i){return i==false;}) && collision_happy==true)
			break;
		Eigen::VectorXd J(mCharacter->GetSkeleton()->getNumDofs());
		J.setZero();
		for(int j=0;j<mMuscleLimitConstraints.size();j++)
		{
			if(active[j])
			{
				auto muscle = mMuscleLimitConstraints[j]->GetMuscle();
				J += muscle->Getdl_dtheta();
			}
		}
		//Gradient Descent while inequalities are not active.
		double alpha = 2.0;
		int k =0;
		for(k=0;k<8;k++){
			Eigen::VectorXd next_pos = cur_pos - alpha*J;
			mCharacter->GetSkeleton()->setPositions(next_pos);
			mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
			bool all_muscle_happy = true;
			for(int j=0;j<mMuscleLimitConstraints.size();j++)
			{
				auto muscle = mMuscleLimitConstraints[j]->GetMuscle();
				if(active[j])
				{

					muscle->Update(0.01);
					if(muscle->l_mt >l_mt[j])
						all_muscle_happy = false;
				}
			}
			if(all_muscle_happy){
				cur_pos = next_pos;
				break;		
			}
			alpha *= 0.5;
		}
		if(k==8&&collision_happy==true)
			break;


		mCharacter->GetSkeleton()->setPositions(cur_pos);
		mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);

		// Eigen::Vector3d p_footl = mCharacter->GetSkeleton()->getBodyNode("TalusL")->getCOM();
		// Eigen::Vector3d p_footr = mCharacter->GetSkeleton()->getBodyNode("TalusR")->getCOM();
		// Eigen::Vector3d p_com = mCharacter->GetSkeleton()->getBodyNode("Pelvis")->getCOM();
		// Eigen::Vector3d projection = p_com - 0.5*(p_footl + p_footr);
		// projection[0] = 0.0;
		// projection.normalize();
		
		// double theta = acos(projection[1]*0.8+projection[2]*0.6);
		// double sin_theta = 0.6*projection[1]-0.8*projection[2];
		// stable_happy = true;
		// std::cout<<theta<<std::endl;
		// // cur_pos[0] = atan2(projection[1],projection[2]);
		// // double cos_theta = projection[1];

		// if(sin_theta>0.0)
		// 	cur_pos[0] = theta;
		// else
		// 	cur_pos[0] = -theta;

		// mCharacter->GetSkeleton()->setPositions(cur_pos);
		// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
		// collision_happy = true;
		// continue;
		//Solve IK to avoid Penentration.	
		auto cg = mWorld->getConstraintSolver()->getCollisionGroup();
		auto co = mWorld->getConstraintSolver()->getCollisionOption();
		auto cr = mWorld->getConstraintSolver()->getLastCollisionResult();
		cr.clear();
		cg->collide(co,&cr);
		// double ground_y = mWorld->GetSkeleton("Ground")->getTransform().translation()[1]+
		// 0.5*dynamic_cast<const dart::dynamics::BoxShape*>(mWorld->GetSkeleton("Ground")->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1];
		bool l_collide,r_collide;
		l_collide = false;
		r_collide = false;

		for (auto i = 0u; i < cr.getNumContacts(); ++i)
		{
			auto& ct = cr.getContact(i);

			auto shapeFrame2 = const_cast<dart::dynamics::ShapeFrame*>(ct.collisionObject2->getShapeFrame());
			std::string name = shapeFrame2->asShapeNode()->getBodyNodePtr()->getName();
			if(name[name.size()-1]=='L')
				l_collide = true;
			else if(name[name.size()-1]=='R')
				r_collide = true;
		}
		if(l_collide==false && r_collide==false)
			collision_happy = true;
		else
			collision_happy = false;
		if(collision_happy)
			continue;
		alpha = 3E-2;
		Eigen::VectorXd next_action(6);
		next_action.setZero();
		if(l_collide)
			next_action[1] = alpha;
		if(r_collide)
			next_action[4] = alpha;

		Eigen::VectorXd next_pos = mCharacter->GetIKTargetPositions(cur_pos,next_action);
		cur_pos = next_pos;
		mCharacter->GetSkeleton()->setPositions(cur_pos);
		mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	
		
	}
	mCharacter->GetSkeleton()->setPositions(cur_pos);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	Eigen::Vector3d p_footl = mCharacter->GetSkeleton()->getBodyNode("TalusL")->getCOM();
	Eigen::Vector3d p_footr = mCharacter->GetSkeleton()->getBodyNode("TalusR")->getCOM();
	Eigen::Vector3d p_com = mCharacter->GetSkeleton()->getBodyNode("Pelvis")->getCOM();
	Eigen::Vector3d projection = p_com - 0.5*(p_footl + p_footr);
	projection[0] = 0.0;
	projection.normalize();
	
	double theta = acos(projection[1]);
	double sin_theta = projection[2];

	if(sin_theta>0.0)
		cur_pos[0] = -theta*0.5;
	else
		cur_pos[0] = theta*0.5;

	mTarget.first = cur_pos;
	// mTarget.second.segment<3>(3).setZero();
	mCharacter->GetSkeleton()->setPositions(mTarget.first);
	mCharacter->GetSkeleton()->setVelocities(mTarget.second);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);

}
bool
Environment::
IsTerminalState()
{
	bool isTerminal = false;

	Eigen::VectorXd p = mCharacter->GetSkeleton()->getPositions();
	Eigen::VectorXd v = mCharacter->GetSkeleton()->getVelocities();
	double root_y = mCharacter->GetSkeleton()->getBodyNode(0)->getTransform().translation()[1];
	Eigen::Vector3d root_v = mCharacter->GetSkeleton()->getBodyNode(0)->getCOMLinearVelocity();

	//ET
	if(root_y<0.7 || root_y > 2.0)
		isTerminal = true;
	if(dart::math::isNan(v)||v.array().abs().maxCoeff()>1E2)
		isTerminal = true;
	if(dart::math::isNan(p))
		isTerminal = true;
	if(mTimeElapsed>40.0)
		isTerminal = true;
	return isTerminal;
}
Eigen::VectorXd
Environment::
GetState()
{
	auto& skel = mCharacter->GetSkeleton();
	dart::dynamics::BodyNode* root = skel->getBodyNode(0);
	auto ibs = mCharacter->GetInterestBodies();
	int num_body_nodes = ibs.size();
	Eigen::VectorXd p,v;

	p.resize(num_body_nodes*3);
	v.resize((num_body_nodes+1)*3);

	for(int i =0;i<num_body_nodes;i++)
	{
		p.segment<3>(3*i) = ibs[i]->getCOM(root);
		v.segment<3>(3*i) = ibs[i]->getCOMLinearVelocity();
	}
	v.tail<3>() = root->getCOMLinearVelocity();

	double phi = mCharacter->GetMotionGraph()->GetPhase();

	p*=0.8;
	v*=0.2;

	Eigen::VectorXd s(p.rows()+v.rows()+1);
	s<<p,v,phi;

	return s;
}
double exp_of_squared(const Eigen::VectorXd& vec,double w = 1.0)
{
	return exp(-w*vec.squaredNorm());
}
double exp_of_squared(const Eigen::Vector3d& vec,double w = 1.0)
{
	return exp(-w*vec.squaredNorm());
}
double exp_of_squared(double val,double w = 1.0)
{
	return exp(-w*val*val);
}
double
Environment::
GetReward()
{
	auto& skel = mCharacter->GetSkeleton();
	Eigen::VectorXd zeros = Eigen::VectorXd::Zero(mCharacter->GetMotionActions().size());
	// auto target = mCharacter->GetTargetPositionsAndVelocitiesFromBVH(mAction.tail(mCharacter->GetMotionActions().size()));
	auto target = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();

	Eigen::VectorXd cur_pos = skel->getPositions();
	Eigen::VectorXd cur_vel = skel->getVelocities();

	Eigen::VectorXd p_diff_all = skel->getPositionDifferences(target.first,cur_pos);
	Eigen::VectorXd v_diff_all = skel->getVelocityDifferences(target.second,cur_vel);
	
	Eigen::VectorXd p_diff = Eigen::VectorXd::Zero(skel->getNumDofs());
	Eigen::VectorXd v_diff = Eigen::VectorXd::Zero(skel->getNumDofs());
	
	auto& bvh_map = mCharacter->GetBVHMap();
	for(auto ss : bvh_map)
	{
		auto joint = mCharacter->GetSkeleton()->getBodyNode(ss.first)->getParentJoint();
		int idx = joint->getIndexInSkeleton(0);
		if(joint->getType()=="FreeJoint")
			continue;
		else if(joint->getType()=="RevoluteJoint")
			p_diff[idx] = p_diff_all[idx];
		else if(joint->getType()=="BallJoint")
			p_diff.segment<3>(idx) = p_diff_all.segment<3>(idx);	
	}
	auto ees = mCharacter->GetEndEffectors();
	Eigen::VectorXd ee_diff(ees.size()*3);
	Eigen::Vector3d com_diff;
	for(int i =0;i<ees.size();i++)
		ee_diff.segment<3>(i*3) = ees[i]->getCOM();
	com_diff = skel->getCOM();
	
	skel->setPositions(target.first);
	skel->computeForwardKinematics(true,false,false);
	for(int i =0;i<ees.size();i++)
		ee_diff.segment<3>(i*3) -= ees[i]->getCOM();
	com_diff -= skel->getCOM();

	skel->setPositions(cur_pos);
	skel->computeForwardKinematics(true,false,false);

	double r_p = exp_of_squared(p_diff,2.0);
	double r_v = exp_of_squared(v_diff,0.1);
	double r_ee = exp_of_squared(ee_diff,40.0);
	double r_com = exp_of_squared(com_diff,10.0);

	double r = w_p*r_p + w_v*r_v + w_ee*r_ee + w_com*r_com;

	if(dart::math::isNan(r))
		return 0.0;
	return r;
}
void
Environment::
SetAction(const Eigen::VectorXd& a)
{
	mAction.head(mCharacter->GetMotionActions().size()) = 0.1*a.head(mCharacter->GetMotionActions().size());
	// mAction.tail(mCharacter->GetMotionActions().size()) = 0.01*a.tail(mCharacter->GetMotionActions().size());
	// auto target = mCharacter->GetTargetPositionsAndVelocitiesFromBVH(mAction.tail(mCharacter->GetMotionActions().size()));


	mTimeElapsed += 1.0 / (double)mControlHz;
	mCharacter->GetMotionGraph()->Step();
	
	mTarget = mCharacter->GetTargetPositionsAndVelocitiesFromBVH(mAction.head(mCharacter->GetMotionActions().size()));
	
	// mMeasurePose[mCharacter->GetSkeleton()->getJoint("FemurL")->getIndexInSkeleton(2)] = 0.7+sin(mTimeElapsed*3.0);
	// mTarget.first = mMeasurePose;
	// mTarget.second.setZero();

	// Eigen::VectorXd IK_action = mAction.tail(mCharacter->GetIKAction()->GetDof());
	// mTarget.first = mCharacter->GetIKTargetPositions(mTarget.first,IK_action);
	mSimCount = 0;
	mRandomSampleIndex = rand()%(mSimulationHz/mControlHz);
	mQP->Update();
	mTempTuple.A = mQP->GetJtA().block(6,0,mCharacter->GetSkeleton()->getNumDofs()-6,mCharacter->GetMuscles().size());
	mTempTuple.b = mQP->GetJtP().segment(6,mCharacter->GetSkeleton()->getNumDofs()-6);

	mTorqueDesired = mCharacter->GetSPDForces(mTarget.first,mTarget.second);
}
}