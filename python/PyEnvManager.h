#ifndef __PY_ENV_MANAGER_H__
#define __PY_ENV_MANAGER_H__
#include "Environment.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "NumPyHelper.h"
class PyEnvManager
{
public:
	PyEnvManager(int num_envs);

	int GetStateDofs();
	int GetActionDofs();
	int GetSystemDofs();
	int GetSimulationHz();
	int GetControlHz();

	void Step(int id);
	void Reset(int id);
	bool IsEndOfEpisode(int id);
	np::ndarray GetState(int id);
	void SetAction(np::ndarray np_array, int id);
	double GetReward(int id);

private:
	std::vector<MSS::Environment*> mEnvs;

	int mNumEnvs;
};

#endif