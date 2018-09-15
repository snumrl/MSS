#ifndef __MSS_PREPROCESS_H__
#define __MSS_PREPROCESS_H__
#include "Environment.h"
#include "WrapperFunctions.h"


class Preprocess
{
public:
	Preprocess(int num_slaves);
	int GetNumDofs(){return mNumDofs;}
	int GetNumMuscles(){return mNumMuscles;}
	p::list GetIndexOffset();
	np::ndarray Get(){return toNumPyArray(mX);};
	np::ndarray GetNormalizer();
	
	p::list GetLinearizedDynamics(np::ndarray p,np::ndarray v);
	void GeneratePairs(int num);
public:
	void GeneratePair(int id,int num);
	std::vector<Eigen::VectorXd> mX;
	std::vector<std::vector<Eigen::VectorXd>> mX_per_slave;
	std::vector<int> mIndexOffset;
	int mNumSlaves;

	int mNumDofs,mNumMuscles;
	std::vector<MSS::Environment*> mSlaves;
};
#endif