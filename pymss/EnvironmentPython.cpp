#include "EnvironmentPython.h"
#include <omp.h>
#include "dart/math/math.hpp"
#include <iostream>

EnvironmentPython::
EnvironmentPython(int num_slaves)
	:mNumSlaves(num_slaves)
{
	dart::math::seedRand();
	omp_set_num_threads(num_slaves);
	for(int i =0;i<mNumSlaves;i++){
		mSlaves.push_back(new MSS::Environment(30,900));
	}
	mNumState = mSlaves[0]->GetNumState();
	mNumAction = mSlaves[0]->GetNumAction();
}
//For general properties
int
EnvironmentPython::
GetNumState()
{
	return mNumState;
}
int
EnvironmentPython::
GetNumAction()
{
	return mNumAction;
}
int
EnvironmentPython::
GetNumDofs()
{
	return mSlaves[0]->GetCharacter()->GetSkeleton()->getNumDofs();
}
int
EnvironmentPython::
GetNumMuscles()
{
	return mSlaves[0]->GetCharacter()->GetMuscles().size();
}
//For each slave
void 
EnvironmentPython::
Step(const Eigen::VectorXd& activation,int id)
{
	mSlaves[id]->Step(activation);
}
void 
EnvironmentPython::
Reset(bool RSI,int id)
{
	mSlaves[id]->Reset(RSI);
}
bool 
EnvironmentPython::
IsTerminalState(int id)
{
	return mSlaves[id]->IsTerminalState();
}
np::ndarray
EnvironmentPython::
GetState(int id)
{
	return toNumPyArray(mSlaves[id]->GetState());
}
void 
EnvironmentPython::
SetAction(np::ndarray np_array,int id)
{
	mSlaves[id]->SetAction(toEigenVector(np_array));
}
double 
EnvironmentPython::
GetReward(int id)
{
	return mSlaves[id]->GetReward();
}

//For all slaves
void
EnvironmentPython::
SetAlpha(double a)
{
	for (int id = 0; id < mNumSlaves; ++id)
		mSlaves[id]->SetAlpha(a);
}
np::ndarray
EnvironmentPython::
GetMuscleTorques()
{
	std::vector<Eigen::VectorXd> mt(mNumSlaves);
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		mt[id] = mSlaves[id]->GetMuscleTorques();
	}
	return toNumPyArray(mt);	
}
np::ndarray
EnvironmentPython::
GetDesiredTorques()
{
	std::vector<Eigen::VectorXd> qdd_des(mNumSlaves);
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		qdd_des[id] = mSlaves[id]->GetDesiredTorques();
	}
	return toNumPyArray(qdd_des);
}
void
EnvironmentPython::
Steps(np::ndarray np_array,p::list _terminated)
{
	std::vector<Eigen::VectorXd> activations =toEigenVectorVector(np_array);
	auto terminated = toStdVector(_terminated);
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		if(terminated[id]==false)
			this->Step(activations[id],id);
	}
}
void
EnvironmentPython::
Resets(bool RSI)
{
	for (int id = 0; id < mNumSlaves; ++id)
	{
		this->Reset(RSI,id);
	}
}
np::ndarray
EnvironmentPython::
IsTerminalStates()
{
	std::vector<bool> is_terminate_vector(mNumSlaves);

	for (int id = 0; id < mNumSlaves; ++id)
		is_terminate_vector[id] = IsTerminalState(id);

	return toNumPyArray(is_terminate_vector);
}
np::ndarray
EnvironmentPython::
GetStates()
{
	Eigen::MatrixXd states(mNumSlaves,mNumState);

	for (int id = 0; id < mNumSlaves; ++id)
		states.row(id) = mSlaves[id]->GetState().transpose();

	return toNumPyArray(states);
}
void
EnvironmentPython::
SetActions(np::ndarray np_array)
{
	Eigen::MatrixXd action = toEigenMatrix(np_array);

	for (int id = 0; id < mNumSlaves; ++id){
		mSlaves[id]->SetAction(action.row(id).transpose());
	}
}
np::ndarray
EnvironmentPython::
GetRewards()
{
	std::vector<float> rewards(mNumSlaves);
	for (int id = 0; id < mNumSlaves; ++id)
		rewards[id] = this->GetReward(id);

	return toNumPyArray(rewards);
}
p::list
EnvironmentPython::
GetTuples()
{
	p::list all;
// #pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		auto& tps = mSlaves[id]->GetTuples();
		for(int j=0;j<tps.size();j++)
		{
			p::list t;
			t.append(toNumPyArray(tps[j].tau));
			t.append(toNumPyArray(tps[j].tau_des));
			t.append(toNumPyArray(tps[j].A));
			t.append(toNumPyArray(tps[j].b));
			all.append(t);
		}
		tps.clear();
	}

	return all;
}
using namespace boost::python;

BOOST_PYTHON_MODULE(pymss)
{
	Py_Initialize();
	np::initialize();

	class_<EnvironmentPython>("Env",init<int>())
		.def("GetNumState",&EnvironmentPython::GetNumState)
		.def("GetNumAction",&EnvironmentPython::GetNumAction)
		.def("GetNumDofs",&EnvironmentPython::GetNumDofs)
		.def("GetNumMuscles",&EnvironmentPython::GetNumMuscles)
		.def("GetNumTotalMuscleRelatedDofs",&EnvironmentPython::GetNumTotalMuscleRelatedDofs)
		.def("GetSimulationHz",&EnvironmentPython::GetSimulationHz)
		.def("GetControlHz",&EnvironmentPython::GetControlHz)
		.def("Reset",&EnvironmentPython::Reset)
		.def("IsTerminalState",&EnvironmentPython::IsTerminalState)
		.def("GetState",&EnvironmentPython::GetState)
		.def("SetAction",&EnvironmentPython::SetAction)
		.def("GetReward",&EnvironmentPython::GetReward)
		.def("SetAlpha",&EnvironmentPython::SetAlpha)
		.def("GetMuscleTorques",&EnvironmentPython::GetMuscleTorques)
		.def("GetDesiredTorques",&EnvironmentPython::GetDesiredTorques)
		.def("Steps",&EnvironmentPython::Steps)
		.def("Resets",&EnvironmentPython::Resets)
		.def("IsTerminalStates",&EnvironmentPython::IsTerminalStates)
		.def("GetStates",&EnvironmentPython::GetStates)
		.def("SetActions",&EnvironmentPython::SetActions)
		.def("GetRewards",&EnvironmentPython::GetRewards)
		.def("GetTuples",&EnvironmentPython::GetTuples);
}