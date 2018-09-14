#include "Preprocess.h"
#include <random>


Preprocess::
Preprocess(int num_slaves)
	:mNumSlaves(num_slaves)
{
	dart::math::seedRand();
	omp_set_num_threads(num_slaves);
	for(int i =0;i<mNumSlaves;i++)
		mSlaves.push_back(new MSS::Environment(30,600));

	mX_per_slave.resize(num_slaves);
	auto character = mSlaves[0]->GetCharacter();	
	mNumInput = character->GetSkeleton()->getNumDofs()*4-12;
	mNumOutput = character->GetMuscles().size();
}
np::ndarray
Preprocess::
GetNormalizer()
{
	int num_dof = mSlaves[0]->GetCharacter()->GetSkeleton()->getNumDofs();
	auto character = mSlaves[0]->GetCharacter();	
	Eigen::VectorXd mean(mNumInput+mNumOutput);
	Eigen::VectorXd std(mNumInput+mNumOutput);
	int count = 0;
	//p
	for(int i =0;i<num_dof-6;i++)
		std[count++] = 1.0;
	
	//v
	for(int i =0;i<num_dof-6;i++)
		std[count++] = 3.0;
	
	//Target p
	for(int i =0;i<num_dof;i++)
		std[count++] = 0.4;

	//Target v
	for(int i =0;i<num_dof;i++)
		std[count++] = 1.0;

	//Target a
	for(int i =0;i<character->GetMuscles().size();i++)
		std[count++] = 2.0;

	count = 0;
	//p
	for(int i =0;i<num_dof-6;i++)
		mean[count++] = 0.0;
	
	//v
	for(int i =0;i<num_dof-6;i++)
		mean[count++] = 0.0;
	
	//Target p
	for(int i =0;i<num_dof;i++)
		mean[count++] = 0.0;

	//Target v
	for(int i =0;i<num_dof;i++)
		mean[count++] = 0.0;

	//Target a
	for(int i =0;i<character->GetMuscles().size();i++)
		mean[count++] = -1.0;

	Eigen::MatrixXd normalizer(2,mNumInput+mNumOutput);
	normalizer.row(0) = mean;
	normalizer.row(1) = std;

	return toNumPyArray(normalizer);
}
void
Preprocess::
GeneratePair(int id,int num)
{
	std::default_random_engine generator;
	std::normal_distribution<double> q_dist(0.0,0.4),q_dot_dist(0.0,1.0);
	
	auto character = mSlaves[id]->GetCharacter();	
	Eigen::VectorXd p_loc = character->GetSkeleton()->getPositions();
	Eigen::VectorXd v_loc = character->GetSkeleton()->getVelocities();
	Eigen::VectorXd p_noise(p_loc.rows());
	Eigen::VectorXd v_noise(v_loc.rows());

	double dt = mSlaves[id]->GetWorld()->getTimeStep();
	Eigen::VectorXd x(mNumInput+mNumOutput);
	
	for(int i=0;i<num;i++)
	{
		for(int k=0;k<p_loc.rows();k++)
		{
			p_noise[k] = q_dist(generator);
			v_noise[k] = q_dot_dist(generator);
		}
		
		Eigen::VectorXd p = p_loc+p_noise;
		Eigen::VectorXd v = v_loc+v_noise;

		character->GetSkeleton()->setPositions(p);
		character->GetSkeleton()->setVelocities(v);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		for(auto muscle : character->GetMuscles())
			muscle->Update(dt);

		auto target = character->GetTargetPositionsAndVelocitiesFromBVH();
		for(int k=0;k<p_loc.rows();k++)
		{
			p_noise[k] = q_dist(generator);
			v_noise[k] = q_dot_dist(generator);
		}
		target.first += p_noise;
		target.second += v_noise;
		Eigen::VectorXd qdd_desired = character->GetSPDAccelerations(target.first,target.second);
		mSlaves[id]->GetQP()->Minimize(qdd_desired);
		Eigen::VectorXd activation = mSlaves[id]->GetQP()->GetSolution().tail(character->GetMuscles().size());

		target.first = target.first - p;
		target.second = target.second - v;
		p = p.tail(p.rows()-6);
		v = v.tail(v.rows()-6);
		x<<p,v,target.first,target.second,activation;
		mX_per_slave[id][i] = x;
	}

}
void
Preprocess::
GeneratePairs(int num)
{
	for(int id =0;id<mNumSlaves;id++)
	{
		mSlaves[id]->Reset(true);
		mX_per_slave[id].clear();
		mX_per_slave[id].resize(num);
	}
	
#pragma omp parallel for
	for(int id =0;id<mNumSlaves;id++)
	{
		GeneratePair(id,num);
	}

	mX.clear();
	for(int id =0;id<mNumSlaves;id++)
		mX.insert( mX.end(), mX_per_slave[id].begin(), mX_per_slave[id].end() );
}
