#include "PyEnvManager.h"
#include <omp.h>

PyEnvManager::
PyEnvManager(int num_envs)
	:mNumEnvs(num_envs)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumEnvs);
	for(int i = 0;i<mNumEnvs;i++)
		mEnvs.push_back(new MSS::Environment());
}
int
PyEnvManager::
GetStateDofs()
{
	return mEnvs[0]->GetStateDofs();
}
int
PyEnvManager::
GetActionDofs()
{
	return mEnvs[0]->GetActionDofs();
}
int
PyEnvManager::
GetSystemDofs()
{
	return mEnvs[0]->GetSystemDofs();
}
int
PyEnvManager::
GetSimulationHz()
{
	return mEnvs[0]->GetSimulationHz();
}
int
PyEnvManager::
GetControlHz()
{
	return mEnvs[0]->GetControlHz();
}
void
PyEnvManager::
Step(int id)
{
	mEnvs[id]->Step();
}
void
PyEnvManager::
Reset(int id)
{
	mEnvs[id]->Reset();
}
bool
PyEnvManager::
IsEndOfEpisode(int id)
{
	return mEnvs[id]->IsEndOfEpisode();
}
np::ndarray 
PyEnvManager::
GetState(int id)
{
	return toNumPyArray(mEnvs[id]->GetState());
}
void 
PyEnvManager::
SetAction(np::ndarray np_array, int id)
{
	mEnvs[id]->SetAction(toEigenVector(np_array));
}
double 
PyEnvManager::
GetReward(int id)
{
	return mEnvs[id]->GetReward();
}


using namespace boost::python;

BOOST_PYTHON_MODULE(pymss)
{
	Py_Initialize();
	np::initialize();

	class_<PyEnvManager>("EnvManager",init<int>())
		.def("GetStateDofs",&PyEnvManager::GetStateDofs)
		.def("GetActionDofs",&PyEnvManager::GetActionDofs)
		.def("GetSystemDofs",&PyEnvManager::GetSystemDofs)
		.def("GetSimulationHz",&PyEnvManager::GetSimulationHz)
		.def("GetControlHz",&PyEnvManager::GetControlHz)
		.def("Step",&PyEnvManager::Step)
		.def("Reset",&PyEnvManager::Reset)
		.def("IsEndOfEpisode",&PyEnvManager::IsEndOfEpisode)
		.def("GetState",&PyEnvManager::GetState)
		.def("SetAction",&PyEnvManager::SetAction)
		.def("GetReward",&PyEnvManager::GetReward);
		// .def("DDD",&PyEnvManager::DDD)
}