#include "PyEnv.h"
#include <omp.h>

PyEnv::
PyEnv(int num_envs)
	:mNumEnvs(num_envs)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumEnvs);
	for(int i = 0;i<mNumEnvs;i++)
		mEnvs.push_back(new MSS::Environment());
}

using namespace boost::python;

BOOST_PYTHON_MODULE(pymss)
{
	Py_Initialize();
	np::initialize();

	class_<PyEnv>("Env",init<int>());
		// .def("DDD",&PyEnv::DDD)
}