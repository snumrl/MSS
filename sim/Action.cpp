#include "Action.h"
namespace MSS
{
MotionAction::
MotionAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower)
	:name(_name),lb(0.0),a(0.5*(upper-lower))
{
	for(int i =0;i<a.rows();i++)
		a[i] = dart::math::clip(a[i],-1.0,1.0);
}
KpAction::
KpAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower)
	:name(_name),lb(0.0),a(0.5*(upper-lower))
{

}
IKAction::
IKAction(const std::string& _name,const dart::dynamics::SkeletonPtr& skeleton,
		const std::vector<dart::dynamics::BodyNode*>& target_body,
		const Eigen::VectorXd& upper_box,const Eigen::VectorXd& lower_box)
	:name(_name),skel(skeleton),target_bn(target_body),a(0.5*(upper_box-lower_box)),num_iteration(1000)
{

}
Eigen::VectorXd
IKAction::
SolveIK(const Eigen::VectorXd& p_old,const Eigen::VectorXd& target)
{
	Eigen::VectorXd p = p_old;

	Eigen::VectorXd gradient = Eigen::VectorXd::Zero(skel->getNumDofs());

	for(int i=0;i<num_iteration;i++)
	{
		gradient.setZero();
		
		double energy_cur = 0.0;
		for(int j=0;j<target.rows()/3;j++)
		{
			dart::math::LinearJacobian J = skel->getLinearJacobian(target_bn[j]);
			Eigen::MatrixXd J_inv = J.transpose()*(J*J.transpose()).inverse();
			Eigen::Vector3d error = target_bn[j]->getCOM() - target.segment<3>(3*j);
			energy_cur += error.squaredNorm();
			gradient += J_inv*error;
		}

		//Line Search

		double alpha =1.0;
		Eigen::VectorXd p_new;
		int k=0;
		double energy_new;
		
		for(k;k<12;k++)
		{
			energy_new = 0.0;
			p_new = p - alpha*gradient;
			skel->setPositions(p_new);
			skel->computeForwardKinematics(true,false,false);
			for(int j=0;j<target.rows()/3;j++)
			{
				Eigen::Vector3d error = target_bn[j]->getCOM() - target.segment<3>(3*j);
				energy_new += error.squaredNorm();
			}
			
			if(energy_new<energy_cur)
				break;
			alpha *= 0.5;
		}
		
		p = p_new;
		skel->setPositions(p);
		skel->computeForwardKinematics(true,false,false);

		if(energy_new<1E-7 || k==12)
			break;
	}
	
	
	return p;
};
void
IKAction::
Set(Eigen::VectorXd& p)
{
	Eigen::VectorXd save_p = skel->getPositions();	
	Eigen::VectorXd target_pos = lb.cwiseProduct(a);
	skel->setPositions(p);
	skel->computeForwardKinematics(true,false,false);
	for(int i=0;i<target_bn.size();i++)
		target_pos.segment<3>(i*3) += target_bn[i]->getCOM();
	Eigen::VectorXd prev = p;
	p = SolveIK(p,target_pos);
	skel->setPositions(save_p);
	skel->computeForwardKinematics(true,false,false);
	
}
};