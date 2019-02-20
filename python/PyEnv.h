#ifndef __PY_ENV_H__
#define __PY_ENV_H__
#include "Environment.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "NumpyHelper.h"
class PyEnv
{
public:
	PyEnv(int num_envs);

private:
	std::vector<MSS::Environment*> mEnvs;

	int mNumEnvs;
};

#endif