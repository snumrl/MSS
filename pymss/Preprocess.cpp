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
	int count = 0;
	mIndexOffset.push_back(count);
	count += mSlaves[0]->GetState().rows();mIndexOffset.push_back(count);							  //s
	count += mSlaves[0]->GetCharacter()->GetSkeleton()->getAccelerations().rows();mIndexOffset.push_back(count); //qdd_des
	// count += mSlaves[0]->GetCharacter()->GetMuscles().size();mIndexOffset.push_back(count);    		 			 //activation
	count += mSlaves[0]->GetCharacter()->GetSkeleton()->getPositions().rows();mIndexOffset.push_back(count);   //q
	count += mSlaves[0]->GetCharacter()->GetSkeleton()->getVelocities().rows();mIndexOffset.push_back(count); //v
	count += mSlaves[0]->GetCharacter()->GetSkeleton()->getPositions().rows();mIndexOffset.push_back(count);   //q_des
	count += mSlaves[0]->GetCharacter()->GetSkeleton()->getVelocities().rows();mIndexOffset.push_back(count);  //v_des
	

	mNumDofs = character->GetSkeleton()->getNumDofs();
	mNumMuscles = character->GetMuscles().size();
}
np::ndarray
Preprocess::
GetNormalizer()
{
	int num_dof = mSlaves[0]->GetCharacter()->GetSkeleton()->getNumDofs();
	auto character = mSlaves[0]->GetCharacter();	
	Eigen::VectorXd mean(mIndexOffset[2]);
	Eigen::VectorXd std(mIndexOffset[2]);
	//Target a
	for(int i=mIndexOffset[0];i<mIndexOffset[1];i++)
		std[i] = 1.0;
	for(int i=mIndexOffset[1];i<mIndexOffset[2];i++)
		std[i] = 1000.0;

	//Target a
	for(int i=mIndexOffset[0];i<mIndexOffset[1];i++)
		mean[i] = 0.0;
	for(int i=mIndexOffset[1];i<mIndexOffset[2];i++)
		mean[i] = 0.0;

	Eigen::MatrixXd normalizer(2,mIndexOffset[2]);
	normalizer.row(0) = mean;
	normalizer.row(1) = std;

	return toNumPyArray(normalizer);
}
p::list
Preprocess::
GetIndexOffset()
{
	p::list io;
	for(int i =0;i<mIndexOffset.size();i++)
		io.append(mIndexOffset[i]);

	return io;
}
// np::ndarray
// Preprocess::
// GetAccelerations(np::ndarray _p,np::ndarray _v,np::ndarray _act)
// {
// 	Eigen::MatrixXd p = toEigenVector(_p);
// 	Eigen::MatrixXd v = toEigenVector(_v);
// 	Eigen::MatrixXd activation = toEigenVector(_act);
// 	Eigen::MatrixXd acc(p.rows(),p.cols());
// 	auto character = mSlaves[0]->GetCharacter();	
// 	double dt = mSlaves[0]->GetWorld()->getTimeStep();
// 	for(int i =0;i<p.rows();i++)
// 	{
// 		Eigen::VectorXd p_i = p.row(i).transpose();
// 		Eigen::VectorXd v_i = v.row(i).transpose();
// 		Eigen::VectorXd a_i = activation.row(i).transpose();
		
// 		character->GetSkeleton()->clearExternalForces();
// 		character->GetSkeleton()->clearInternalForces();
// 		character->GetSkeleton()->setPositions(p_i);
// 		character->GetSkeleton()->setVelocities(v_i);
// 		character->GetSkeleton()->computeForwardKinematics(true,false,false);

// 		int count = 0;
// 		for(auto muscle : character->GetMuscles())
// 		{
// 			muscle->activation = a_i[count++];
// 			muscle->Update(dt);
// 			muscle->ApplyForceToBody();
// 		}
		
// 		character->GetSkeleton()->computeForwardDynamics();
// 		Eigen::VectorXd acc_i = character->GetSkeleton()->getAccelerations();
// 		acc.row(i) = acc_i.transpose();
// 	}
// 	return toNumPyArray(acc);
// }
p::list
Preprocess::
GetLinearizedDynamics(np::ndarray _p,np::ndarray _v)
{
	p::list Ab;
	Eigen::MatrixXd p = toEigenMatrix(_p);
	Eigen::MatrixXd v = toEigenMatrix(_v);
	int n = p.rows();
	std::vector<Eigen::MatrixXd> A(n);
	std::vector<Eigen::VectorXd> b(n);
	Eigen::VectorXd zeros = p.row(0).transpose();
	auto character = mSlaves[0]->GetCharacter();	
	double dt = mSlaves[0]->GetWorld()->getTimeStep();
	for(int i =0;i<n;i++)
	{
		Eigen::VectorXd p_i = p.row(i).transpose();
		Eigen::VectorXd v_i = v.row(i).transpose();
		
		character->GetSkeleton()->clearExternalForces();
		character->GetSkeleton()->clearInternalForces();
		character->GetSkeleton()->setPositions(p_i);
		character->GetSkeleton()->setVelocities(v_i);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);

		int count = 0;
		for(auto muscle : character->GetMuscles())
			muscle->Update(dt);
		
		mSlaves[0]->GetQP()->Update(zeros);
		Eigen::MatrixXd M_inv = character->GetSkeleton()->getInvMassMatrix();
		A[i] = M_inv*mSlaves[0]->GetQP()->GetJtA();
		b[i] = M_inv*mSlaves[0]->GetQP()->GetJtp_minus_c();
	}
	Ab.append(toNumPyArray(A));
	Ab.append(toNumPyArray(b));
	return Ab;

}
void
Preprocess::
GeneratePair(int id,int num)
{
	std::default_random_engine generator;
	std::normal_distribution<double> q_dist(0.0,0.1),q_dot_dist(0.0,0.3);
	
	auto character = mSlaves[id]->GetCharacter();	
	Eigen::VectorXd p_loc = character->GetSkeleton()->getPositions();
	Eigen::VectorXd v_loc = character->GetSkeleton()->getVelocities();
	Eigen::VectorXd p_noise(p_loc.rows());
	Eigen::VectorXd v_noise(v_loc.rows());

	double dt = mSlaves[id]->GetWorld()->getTimeStep();
	Eigen::VectorXd x(mIndexOffset.back());
	
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
		Eigen::VectorXd s = mSlaves[id]->GetState();
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
		// mSlaves[id]->GetQP()->Minimize(qdd_desired);
		// Eigen::VectorXd activation = mSlaves[id]->GetQP()->GetSolution().tail(character->GetMuscles().size());

		target.first = target.first;
		target.second = target.second;
		x<<s,qdd_desired,p,v,target.first,target.second;
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
