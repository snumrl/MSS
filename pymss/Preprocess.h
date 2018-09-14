#ifndef __MSS_PREPROCESS_H__
#define __MSS_PREPROCESS_H__
#include "Environment.h"
#include "WrapperFunctions.h"
class Preprocess
{
public:
	Preprocess(int num_slaves);
	int GetNumInput(){return mNumInput;};
	int GetNumOutput(){return mNumOutput;};
	np::ndarray Get(){return toNumPyArray(mX);};
	np::ndarray GetNormalizer();
	void GeneratePairs(int num);
public:
	void GeneratePair(int id,int num);
	std::vector<Eigen::VectorXd> mX;
	std::vector<std::vector<Eigen::VectorXd>> mX_per_slave;
	int mNumSlaves;
	int mNumInput,mNumOutput;
	std::vector<MSS::Environment*> mSlaves;
};
#endif