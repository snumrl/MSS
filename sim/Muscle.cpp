#include "Muscle.h"

namespace MSS
{

Muscle::
Muscle(std::string _name,double _f0,double _lm0,double _lt0,double _pen_angle)
	:name(_name),f0(_f0),l_m0(_lm0),l_t0(_lt0),l_mt0(0.0),l_mt(1.0),activation(0.0),f_toe(0.33),k_toe(3.0),k_lin(51.878788),e_toe(0.02),e_t0(0.033),k_pe(4.0),e_mo(0.6),gamma(0.5)
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

	Update(0.1);
	Eigen::MatrixXd Jt = GetJacobianTranspose();
	auto Ap = GetForceJacobianAndPassive();
	Eigen::VectorXd JtA = Jt*Ap.first;
	num_related_dofs = 0;
	related_dof_indices.clear();
	for(int i =0;i<JtA.rows();i++)
		if(std::abs(JtA[i])>1E-5){
			num_related_dofs++;
			related_dof_indices.push_back(i);
		}
}
void
Muscle::
ApplyForceToBody()
{
	double f = GetForce();

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

	double x = l_mt - l_t0;

	double f,f_plus,f_minus,f_next,grad,alpha,x_next;
	for(int i =0;i<100;i++)
	{
		f = g(x);
		f_plus = g(x+0.01);
		f_minus = g(x-0.01);
		grad = f/((f_plus-f_minus)*50.0);
		alpha = 1.0;
		if(std::abs(f)<1E-4)
			break;
		for(int j =0;j<8;j++)
		{
			x_next = x - alpha*grad;
			f_next = g(x_next);

			if(std::abs(f_next)<std::abs(f))
				break;
			alpha= 0.1*alpha;
		}
	}
	l_m = x;
	// Eigen::MatrixXd Jt = GetJacobianTranspose();
	// Eigen::VectorXd A = GetForceJacobianAndPassive().second;
	// A = Jt*A;
	// std::cout<<name<<" : "<<A.transpose()<<std::endl;
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
	return f0*g_al(l_m/l_m0);
}
double
Muscle::
Getf_p()
{
	return f0*g_pl(l_m/l_m0);
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
Eigen::VectorXd
Muscle::
GetRelatedJtA()
{
	Eigen::MatrixXd Jt = GetJacobianTranspose();
	
	Eigen::VectorXd A = GetForceJacobianAndPassive().first;

	Eigen::VectorXd JtA = Jt*A;
	
	Eigen::VectorXd JtA_reduced = Eigen::VectorXd::Zero(num_related_dofs);
	for(int i =0;i<num_related_dofs;i++){
		JtA_reduced[i] = JtA[related_dof_indices[i]];
	}
	return JtA_reduced;
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
double
Muscle::
g(double _l_m)
{
	double e_t = (l_mt -_l_m-l_t0)/l_t0;
	_l_m = _l_m/l_m0;
	double f = g_t(e_t) - (g_pl(_l_m)+activation*g_al(_l_m));
	return f;
}
double
Muscle::
g_t(double e_t)
{
	double f_t;
	if(e_t<=e_t0)
		f_t = f_toe/(exp(k_toe)-1)*(exp(k_toe*e_t/e_toe)-1);
	else
		f_t = k_lin*(e_t-e_toe)+f_toe;

	return f_t;
}
double
Muscle::
g_pl(double _l_m)
{
	double f_pl = (exp(k_pe*(_l_m-1.0)/e_mo)-1.0)/(exp(k_pe)-1.0);
	if(_l_m<1.0)
		return 0.0;
	else
		return f_pl;
}
double
Muscle::
g_al(double _l_m)
{
	return exp(-(_l_m-1.0)*(_l_m-1.0)/gamma);
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