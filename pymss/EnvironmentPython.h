#ifndef __MSS_ENVIRONMENT_PYTHON_H__
#define __MSS_ENVIRONMENT_PYTHON_H__
#include "Environment.h"
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "WrapperFunctions.h"

class EnvironmentPython
{
public:
	EnvironmentPython(int num_slaves);
	//For general properties
	int GetNumState();
	int GetNumAction();

	//For each slave
	void Step(int id);
	void Reset(bool RSI,int id);
	bool IsTerminalState(int id);
	np::ndarray GetState(int id);
	void SetAction(np::ndarray np_array,int id);
	double GetReward(int id);

	//For all slaves
	void Steps();
	void Resets(bool RSI);
	np::ndarray IsTerminalStates();
	np::ndarray GetStates();
	void SetActions(np::ndarray np_array);
	np::ndarray GetRewards();
private:
	std::vector<MSS::Environment*> mSlaves;

	int mNumSlaves;
	int mNumState;
	int mNumAction;
};
#endif