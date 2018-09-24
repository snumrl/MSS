#include "Environment.h"
#include "SkeletonBuilder.h"
#include <initializer_list>
#include <chrono>
namespace MSS
{

Environment::
Environment(int control_Hz,int simulation_Hz)
	:mControlHz(control_Hz),mSimulationHz(simulation_Hz),mWorld(std::make_shared<dart::simulation::World>()),w_p(0.65),w_v(0.1),w_ee(0.15),w_com(0.1)
{
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0));
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	// this->mWorld->getConstraintSolver()->setLCPSolver(dart::common::make_unique<dart::constraint::PGSLCPSolver>(mWorld->getTimeStep()));
	mGround = MSS::SkeletonBuilder::BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/character/ground.xml"));
	mWorld->addSkeleton(mGround);
	mCharacter = new Character(mWorld,std::string(MSS_ROOT_DIR)+std::string("/character/full_body.xml"));
	mWorld->addSkeleton(mCharacter->GetSkeleton());
	// mWorld->getConstraintSolver()->addConstraint(std::make_shared<dart::constraint::WeldJointConstraint>(mCharacter->GetSkeleton()->getBodyNode(0)));
	mCharacter->LoadMuscles(std::string(MSS_ROOT_DIR)+std::string("/character/muscle.xml"));
	std::cout<<"# muscles : "<<mCharacter->GetMuscles().size()<<std::endl;
	//Hard-coded parameter
	// mCharacter->LoadContactPoints(std::string(MSS_ROOT_DIR)+std::string("/character/txt/contact_simple.txt"),0.035,mGround->getBodyNode(0));
	
	mCharacter->LoadMotionGraph(std::string(MSS_ROOT_DIR)+std::string("/motion/simple.graph"),std::vector<int>{0,0,0,0,0},1.0/(double)mControlHz);
	mCharacter->AddInterestBodies(std::vector<std::string> {"R_Femur","R_Tibia","R_Talus"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"L_Femur","L_Tibia","L_Talus"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"Spine","Torso","Neck","Skull"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"R_Shoulder","R_Humerus","R_Radius","R_Hand"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"L_Shoulder","L_Humerus","L_Radius","L_Hand"});
	mCharacter->AddEndEffector("R_Talus");
	mCharacter->AddEndEffector("L_Talus");
	mCharacter->AddEndEffector("R_Hand");
	mCharacter->AddEndEffector("L_Hand");
	mCharacter->AddEndEffector("Skull");

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
		std::string name = "R_Femur";
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
		std::string name = "L_Femur";
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
		std::string name = "R_Tibia";
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
		std::string name = "L_Tibia";
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
		std::string name = "R_Talus";
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
		std::string name = "L_Talus";
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
		std::string name = "Skull";
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
		std::string name = "R_Shoulder";
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
		std::string name = "L_Shoulder";
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
		std::string name = "R_Humerus";
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
		std::string name = "L_Humerus";
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
		std::string name = "R_Radius";
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
		std::string name = "L_Radius";
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
		std::string name = "R_Hand";
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
		std::string name = "L_Hand";
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
		// std::vector<std::string> target_name = {"Pelvis","TalusR","TalusL"};
		std::vector<std::string> target_name = {};
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
	Reset(false);
	
}
void
Environment::
Step(const Eigen::VectorXd& activation)
{
	//For Kinematic Moves
	// Eigen::VectorXd activation = GetActivationFromNN();
	// int count = 0;
	// for(auto muscle : mCharacter->GetMuscles())
	// {
	// 	muscle->activation = activation[count++];
	// 	muscle->Update(mWorld->getTimeStep());
	// }
	// mCharacter->GetSkeleton()->setPositions(target.first);
	// mCharacter->GetSkeleton()->setVelocities(target.second);
	// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	// return;

	//For Muscle Actuator

	
	// int count = 0;
	// for(auto muscle : mCharacter->GetMuscles())
	// {
	// 	muscle->activation = activation[count++];
	// 	muscle->Update(mWorld->getTimeStep());
	// 	muscle->ApplyForceToBody();
	// }
	//For Joint Torque
	Eigen::VectorXd tau = mCharacter->GetSkeleton()->getMassMatrix()*mQddDesired+mCharacter->GetSkeleton()->getCoriolisAndGravityForces();
	mCharacter->GetSkeleton()->setForces(tau);
	Eigen::MatrixXd M_inv = mCharacter->GetSkeleton()->getInvMassMatrix();
	Tuple tp;
	tp.s = GetState();
	tp.qdd_des = mQddDesired;
	tp.activation = activation;
	tp.A = M_inv*mQP->GetJtA();
	tp.b = M_inv*mQP->GetJtp_minus_c();

	mTuples.push_back(tp);
	auto contact_points = mCharacter->GetContactPoints();
	for(auto cp : contact_points)
	{
		cp->CheckColliding();
		if(cp->IsColliding()){
			cp->Add(mWorld);
		}
	}
	mWorld->step();
	for(auto cp : contact_points)
		cp->Remove(mWorld);
}
Eigen::VectorXd
Environment::
ComputeActivationQP()
{
	// for(auto muscle : mCharacter->GetMuscles())
		// muscle->Update(mWorld->getTimeStep());
	mQddDesired = mCharacter->GetSPDAccelerations(mTarget.first,mTarget.second);
	// mQP->Minimize(mQddDesired);

	Eigen::VectorXd solution = mQP->GetSolution();

	return solution.tail(mCharacter->GetMuscles().size());
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
	mTarget = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();
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
	if(dart::math::isNan(v))
		isTerminal = true;
	if(dart::math::isNan(p))
		isTerminal = true;
	if(mTimeElapsed>mCharacter->GetMotionGraph()->GetMaxTime())
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
	mAction = 0.1*a;
	mTimeElapsed += 1.0 / (double)mControlHz;
	mCharacter->GetMotionGraph()->Step();
	
	mTarget = mCharacter->GetTargetPositionsAndVelocitiesFromBVH(mAction);
}
}