#ifndef __MSS_MODE_H__
#define __MSS_MODE_H__
#include "dart/dart.hpp"
namespace MSS
{
class MotionAction
{
public:
	std::string name;
	double lb;
	Eigen::VectorXd a;

	MotionAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower);
	void SetLB(double _lb){lb = _lb;};
	void Set(Eigen::VectorXd& p){p+=lb*a;};
};

class KpAction
{
public:
	std::string name;
	double lb;
	Eigen::VectorXd a;
	KpAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower);
	void SetLB(double _lb){lb = _lb;};
	void Set(Eigen::VectorXd& p){p+=lb*a;};
};

class IKAction
{
public:
	dart::dynamics::SkeletonPtr skel;
	Eigen::VectorXd a;
	Eigen::VectorXd lb;
	std::string name;
	
	std::vector<dart::dynamics::BodyNode*> target_bn;
	int num_iteration;
	IKAction(const std::string& _name,const dart::dynamics::SkeletonPtr& skeleton,
		const std::vector<dart::dynamics::BodyNode*>& target_body,
		const Eigen::VectorXd& upper_box,const Eigen::VectorXd& lower_box);
	int GetDof(){return target_bn.size()*3;}
	void SetLB(const Eigen::VectorXd& _lb){lb = _lb;};
	Eigen::VectorXd SolveIK(const Eigen::VectorXd& p_old,const Eigen::VectorXd& target);
	void Set(Eigen::VectorXd& p);
};

};


#endif
