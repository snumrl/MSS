#include "EnvironmentPython.h"
#include <omp.h>
#include "dart/math/math.hpp"
#include <iostream>

EnvironmentPython::
EnvironmentPython(int num_slaves,std::string muscle_nn_path)
	:mNumSlaves(num_slaves)
{
	dart::math::seedRand();
	omp_set_num_threads(num_slaves);
	for(int i =0;i<mNumSlaves;i++){
		mSlaves.push_back(new MSS::Environment(30,600));
		mSlaves[i]->LoadMuscleNN(muscle_nn_path);
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
//For each slave
void 
EnvironmentPython::
Step(int id)
{
	mSlaves[id]->Step();
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
Steps()
{
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		this->Step(id);
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

	for (int id = 0; id < mNumSlaves; ++id)
		mSlaves[id]->SetAction(action.row(id).transpose());
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

using namespace boost::python;

BOOST_PYTHON_MODULE(pymss)
{
	Py_Initialize();
	np::initialize();
	class_<Preprocess>("Preprocess",init<int>())
		.def("GeneratePairs",&Preprocess::GeneratePairs)
		.def("GetNumDofs",&Preprocess::GetNumDofs)
		.def("GetNumMuscles",&Preprocess::GetNumMuscles)
		.def("GetIndexOffset",&Preprocess::GetIndexOffset)
		.def("GetNormalizer",&Preprocess::GetNormalizer)
		.def("GetLinearizedDynamics",&Preprocess::GetLinearizedDynamics)
		.def("Get",&Preprocess::Get);

	class_<EnvironmentPython>("Env",init<int,std::string>())
		.def("GetNumState",&EnvironmentPython::GetNumState)
		.def("GetNumAction",&EnvironmentPython::GetNumAction)
		.def("Step",&EnvironmentPython::Step)
		.def("Reset",&EnvironmentPython::Reset)
		.def("IsTerminalState",&EnvironmentPython::IsTerminalState)
		.def("GetState",&EnvironmentPython::GetState)
		.def("SetAction",&EnvironmentPython::SetAction)
		.def("GetReward",&EnvironmentPython::GetReward)
		.def("Steps",&EnvironmentPython::Steps)
		.def("Resets",&EnvironmentPython::Resets)
		.def("IsTerminalStates",&EnvironmentPython::IsTerminalStates)
		.def("GetStates",&EnvironmentPython::GetStates)
		.def("SetActions",&EnvironmentPython::SetActions)
		.def("GetRewards",&EnvironmentPython::GetRewards);
}