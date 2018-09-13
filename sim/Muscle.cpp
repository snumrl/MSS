#include "Muscle.h"

namespace MSS
{

Muscle::
Muscle(std::string _name,double _f0,double _lm0,double _lt0,double _pen_angle)
	:name(_name),f0(_f0),l_mt0(0.0),l_mt(1.0),activation(0.0)
{

}
void
Muscle::
AddAnchor(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& glob_pos)
{
	Eigen::Vector3d local_pos = bn->getTransform().inverse()*glob_pos;
	mAnchors.push_back(std::make_pair(bn,local_pos));
	int n =mAnchors.size();
	if(n>1)
		l_mt0 += (GetPoint(mAnchors[n-1])-GetPoint(mAnchors[n-2])).norm();
}
void
Muscle::
ApplyForceToBody()
{
	double f = GetForce();
	// std::cout<<f<<std::endl;
	std::vector<Eigen::Vector3d> point;
	for(int i =0;i<mAnchors.size();i++)
		point.push_back(GetPoint(mAnchors[i]));
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = point[i+1]-point[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}

	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = point[i-1]-point[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}
}
void
Muscle::
Update(double time_step)
{
	l_mt = Getl_mt();
}
double
Muscle::
GetForce()
{
	return Getf_A()*activation + Getf_p();
}
double
Muscle::
Getf_A()
{
	return f0;
}
double
Muscle::
Getf_p()
{
	double f_p = 0.0;
	if(l_mt>1.2)
		f_p = 0.4*(l_mt-1.2);

	return f0*f_p;
}
double
Muscle::
Getl_mt()
{
	l_mt = 0.0;
	for(int i=1;i<mAnchors.size();i++)
		l_mt += (GetPoint(mAnchors[i])-GetPoint(mAnchors[i-1])).norm();

	return l_mt/l_mt0;
}
Eigen::MatrixXd
Muscle::
GetJacobianTranspose()
{
	const auto& skel = mAnchors[0].first->getSkeleton();
	int dof = skel->getNumDofs();
	Eigen::MatrixXd Jt(dof,3*mAnchors.size());

	Jt.setZero();
	for(int i =0;i<mAnchors.size();i++)
		Jt.block(0,i*3,dof,3) = skel->getLinearJacobian(mAnchors[i].first,mAnchors[i].second).transpose();
	
	return Jt;	
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Muscle::
GetForceJacobianAndPassive()
{
	double f_a = Getf_A();
	double f_p = Getf_p();
	// std::cout<<name<<":"<<std::endl;
	// std::cout<<f_a<<std::endl;
	// std::cout<<f_p<<std::endl;
	std::vector<Eigen::Vector3d> point,force_dir;
	for(int i =0;i<mAnchors.size();i++){
		point.push_back(GetPoint(mAnchors[i]));
		force_dir.push_back(Eigen::Vector3d::Zero());
	}
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = point[i+1]-point[i];
		dir.normalize();
		force_dir[i] += dir;
	}

	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = point[i-1]-point[i];
		dir.normalize();
		force_dir[i] += dir;
	}

	Eigen::VectorXd A(3*mAnchors.size());
	Eigen::VectorXd p(3*mAnchors.size());
	A.setZero();
	p.setZero();

	for(int i =0;i<mAnchors.size();i++)
	{
		A.segment<3>(i*3) = force_dir[i]*f_a;
		p.segment<3>(i*3) = force_dir[i]*f_p;
	}

	return std::make_pair(A,p);
}
Eigen::Vector3d GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos)
{
	return bn->getTransform()*local_pos;
}
Eigen::Vector3d GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos)
{
	return bnpos.first->getTransform()*bnpos.second;
}
};

