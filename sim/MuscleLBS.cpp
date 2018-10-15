#include "MuscleLBS.h"

using namespace MSS;
LBSAnchor::
LBSAnchor(std::vector<dart::dynamics::BodyNode*> bns,std::vector<Eigen::Vector3d> lps,std::vector<double> ws)
	:bodynodes(bns),local_positions(lps),weights(ws),num_related_bodies(bns.size())
{

}

Eigen::Vector3d
LBSAnchor::
GetPoint()
{
	Eigen::Vector3d p;
	p.setZero();
	for(int i = 0;i<num_related_bodies;i++)
		p += weights[i]*(bodynodes[i]->getTransform()*local_positions[i]);
	return p;
}

MuscleLBS::
MuscleLBS(std::string _name,double _f0,double _lm0,double _lt0,double _pen_angle,double lmax)
	:name(_name),f0(_f0),l_m0(_lm0),l_m(l_mt - l_t0),l_t0(_lt0),l_mt0(0.0),l_mt(1.0),activation(0.0),f_toe(0.33),k_toe(3.0),k_lin(51.878788),e_toe(0.02),e_t0(0.033),k_pe(4.0),e_mo(0.6),gamma(0.5),l_mt_max(lmax)
{
}
std::vector<int> sort_indices(const std::vector<double>& val)
{
	std::vector<int> idx(val.size());
	std::iota(idx.begin(),idx.end(),0);

	std::sort(idx.begin(),idx.end(),[&val](int i1,int i2){return val[i1]<val[i2];});

	return idx;
}
void
MuscleLBS::
AddAnchor(const dart::dynamics::SkeletonPtr& skel,dart::dynamics::BodyNode* bn,const Eigen::Vector3d& glob_pos,int num_related_bodies)
{
	std::vector<double> distance;
	std::vector<Eigen::Vector3d> local_positions;
	distance.resize(skel->getNumBodyNodes(),0.0);
	local_positions.resize(skel->getNumBodyNodes());
	for(int i =0;i<skel->getNumBodyNodes();i++)
	{
		Eigen::Isometry3d T;
		T = skel->getBodyNode(i)->getTransform()*skel->getBodyNode(i)->getParentJoint()->getTransformFromChildBodyNode();
		// local_positions[i] = glob_pos-skel->getBodyNode(i)->getTransform().translation();
		// local_positions[i] = glob_pos-skel->getBodyNode(i)->getTransform().translation();
		local_positions[i] = skel->getBodyNode(i)->getTransform().inverse()*glob_pos;
		distance[i] = (glob_pos-T.translation()).norm();
		// distance[i] = local_positions[i].norm();
		// std::cout<<skel->getBodyNode(i)->getName()<<" "<<distance[i]<<std::endl;
	}
	std::vector<int> index_sort_by_distance = sort_indices(distance);
	// std::cout<<std::endl;
	// for(int i =0;i<skel->getNumBodyNodes();i++)
	// {
	// 	std::cout<<skel->getBodyNode(index_sort_by_distance[i])->getName()<<" "<<distance[index_sort_by_distance[i]]<<std::endl;
	// }
	// std::cout<<std::endl;
	
	std::vector<dart::dynamics::BodyNode*> lbs_body_nodes;
	std::vector<Eigen::Vector3d> lbs_local_positions;
	std::vector<double> lbs_weights;

	// lbs_body_nodes.resize(num_related_bodies);
	// lbs_local_positions.resize(num_related_bodies);
	// lbs_weights.resize(num_related_bodies);
		
	
	double total_weight = 0.0;
	// if(false)
	if(distance[index_sort_by_distance[0]]<0.08)
	{
		lbs_weights.push_back(1.0/sqrt(distance[index_sort_by_distance[0]]));
		total_weight += lbs_weights[0];
		lbs_body_nodes.push_back(skel->getBodyNode(index_sort_by_distance[0]));
		lbs_local_positions.push_back(local_positions[index_sort_by_distance[0]]);

		if(lbs_body_nodes[0]->getParentBodyNode()!=nullptr)
		{
			auto bn_parent = lbs_body_nodes[0]->getParentBodyNode();
			lbs_weights.push_back(1.0/sqrt(distance[bn_parent->getIndexInSkeleton()]));
			total_weight += lbs_weights[1];
			lbs_body_nodes.push_back(bn_parent);
			lbs_local_positions.push_back(local_positions[bn_parent->getIndexInSkeleton()]);
		}	
	}
	else
	{
		total_weight = 1.0;
		lbs_weights.push_back(1.0);
		lbs_body_nodes.push_back(bn);
		lbs_local_positions.push_back(bn->getTransform().inverse()*glob_pos);
	}
	
	
	// for(int i = 0;i<num_related_bodies;i++)
	// {
	// 	lbs_weights[i] = 1.0/distance[index_sort_by_distance[i]];
		

	// 	total_weight += lbs_weights[i];
	// 	lbs_body_nodes[i] = skel->getBodyNode(index_sort_by_distance[i]);
	// 	lbs_local_positions[i] = local_positions[index_sort_by_distance[i]];
	// 	std::cout<<lbs_body_nodes[i]->getName()<<" "<<distance[index_sort_by_distance[i]]<<std::endl;
	// }

	for(int i = 0;i < lbs_body_nodes.size();i++){

		lbs_weights[i] /= total_weight;
	}
	// for(int i = 0;i<lbs_body_nodes.size();i++)
		// std::cout<<lbs_body_nodes[i]->getName()<<" "<<lbs_weights[i]<<std::endl;
	// std::cout<<std::endl<<std::endl<<std::endl<<std::endl;
	mAnchors.push_back(new LBSAnchor(lbs_body_nodes,lbs_local_positions,lbs_weights));

	int n =mAnchors.size();
	if(n>1)
		l_mt0 += (mAnchors[n-1]->GetPoint()-mAnchors[n-2]->GetPoint()).norm();

	mCachedAnchorPositions.resize(n);
	Update(0.1);
	Eigen::MatrixXd Jt = GetJacobianTranspose();
	auto Ap = GetForceJacobianAndPassive();
	Eigen::VectorXd JtA = Jt*Ap.first;
	num_related_dofs = 0;
	related_dof_indices.clear();
	for(int i =0;i<JtA.rows();i++)
		if(std::abs(JtA[i])>1E-3){
			num_related_dofs++;
			related_dof_indices.push_back(i);
		}
}
void
MuscleLBS::
AddAnchor(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& glob_pos)
{
	std::vector<dart::dynamics::BodyNode*> lbs_body_nodes;
	std::vector<Eigen::Vector3d> lbs_local_positions;
	std::vector<double> lbs_weights;

	lbs_body_nodes.push_back(bn);
	lbs_local_positions.push_back(bn->getTransform().inverse()*glob_pos);
	lbs_weights.push_back(1.0);

	mAnchors.push_back(new LBSAnchor(lbs_body_nodes,lbs_local_positions,lbs_weights));

	int n =mAnchors.size();
	if(n>1)
		l_mt0 += (mAnchors[n-1]->GetPoint()-mAnchors[n-2]->GetPoint()).norm();

	mCachedAnchorPositions.resize(n);
	Update(0.1);
	Eigen::MatrixXd Jt = GetJacobianTranspose();
	auto Ap = GetForceJacobianAndPassive();
	Eigen::VectorXd JtA = Jt*Ap.first;
	num_related_dofs = 0;
	related_dof_indices.clear();
	for(int i =0;i<JtA.rows();i++)
		if(std::abs(JtA[i])>1E-3){
			num_related_dofs++;
			related_dof_indices.push_back(i);
		}
	
}
void
MuscleLBS::
ApplyForceToBody()
{
	double f = GetForce();

	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = mCachedAnchorPositions[i+1]-mCachedAnchorPositions[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i]->bodynodes[0]->addExtForce(dir,mCachedAnchorPositions[i],false,false);
	}

	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = mCachedAnchorPositions[i-1]-mCachedAnchorPositions[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i]->bodynodes[0]->addExtForce(dir,mCachedAnchorPositions[i],false,false);
	}
}
void
MuscleLBS::
Update(double time_step)
{
	for(int i =0;i<mAnchors.size();i++)
		mCachedAnchorPositions[i] = mAnchors[i]->GetPoint();
	l_mt = Getl_mt();

	double x = l_mt - l_t0;
	
	// double x = l_m;

	double f,f_plus,f_minus,f_next,grad,alpha,x_next;
	for(int i =0;i<5;i++)
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
MuscleLBS::
GetForce()
{
	return Getf_A()*activation + Getf_p();
}
double
MuscleLBS::
Getf_A()
{
	return f0*g_al(l_m/l_m0);
}
double
MuscleLBS::
Getf_p()
{
	return f0*g_pl(l_m/l_m0);
}
double
MuscleLBS::
Getl_mt()
{
	l_mt = 0.0;
	for(int i=1;i<mAnchors.size();i++)
		l_mt += (mCachedAnchorPositions[i]-mCachedAnchorPositions[i-1]).norm();

	return l_mt/l_mt0;
}
Eigen::VectorXd
MuscleLBS::
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
MuscleLBS::
GetJacobianTranspose()
{
	const auto& skel = mAnchors[0]->bodynodes[0]->getSkeleton();
	int dof = skel->getNumDofs();
	Eigen::MatrixXd Jt(dof,3*mAnchors.size());

	Jt.setZero();
	for(int i =0;i<mAnchors.size();i++)
		Jt.block(0,i*3,dof,3) = skel->getLinearJacobian(mAnchors[i]->bodynodes[0],mAnchors[i]->bodynodes[0]->getTransform().inverse()*mCachedAnchorPositions[i]).transpose();
	
	return Jt;	
}

std::pair<Eigen::VectorXd,Eigen::VectorXd>
MuscleLBS::
GetForceJacobianAndPassive()
{
	double f_a = Getf_A();
	double f_p = Getf_p();

	std::vector<Eigen::Vector3d> force_dir;
	for(int i =0;i<mAnchors.size();i++){
		force_dir.push_back(Eigen::Vector3d::Zero());
	}
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = mCachedAnchorPositions[i+1]-mCachedAnchorPositions[i];
		dir.normalize();
		force_dir[i] += dir;
	}
	
	
	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = mCachedAnchorPositions[i-1]-mCachedAnchorPositions[i];
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

std::vector<dart::dynamics::Joint*>
MuscleLBS::
GetRelatedJoints()
{
	auto skel = mAnchors[0]->bodynodes[0]->getSkeleton();
	std::map<dart::dynamics::Joint*,int> jns;
	std::vector<dart::dynamics::Joint*> jns_related;
	for(int i =0;i<skel->getNumJoints();i++)
		jns.insert(std::make_pair(skel->getJoint(i),0));
	ComputeJacobians();
	//Compute dL/dtheta
	Eigen::VectorXd dl_dtheta(skel->getNumDofs());
	dl_dtheta.setZero();
	Update(0.01);
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d li = mCachedAnchorPositions[i+1] - mCachedAnchorPositions[i];
		Eigen::MatrixXd dp_i1_minus_dpi = mCachedJs[i+1]-mCachedJs[i];
		Eigen::VectorXd d_li_d_theta = 2*dp_i1_minus_dpi.transpose()*li;
		dl_dtheta += d_li_d_theta;
	}
	
	for(int i =0;i<dl_dtheta.rows();i++)
		if(std::abs(dl_dtheta[i])>1E-6)
		{
			jns[skel->getDof(i)->getJoint()]+=1;
		}

	for(auto jn : jns)
		if(jn.second>0)
			jns_related.push_back(jn.first);
	return jns_related;
}
std::vector<dart::dynamics::BodyNode*>
MuscleLBS::
GetRelatedBodyNodes()
{
	std::vector<dart::dynamics::BodyNode*> bns_related;
	auto rjs = GetRelatedJoints();
	for(auto joint : rjs){
		bns_related.push_back(joint->getChildBodyNode());
	}

	return bns_related;
}
void
MuscleLBS::
ComputeJacobians()
{
	const auto& skel = mAnchors[0]->bodynodes[0]->getSkeleton();
	int dof = skel->getNumDofs();
	mCachedJs.resize(mAnchors.size());
	for(int i =0;i<mAnchors.size();i++)
	{
		mCachedJs[i].resize(3,skel->getNumDofs());
		mCachedJs[i].setZero();

		for(int j=0;j<mAnchors[i]->num_related_bodies;j++){
			mCachedJs[i] += mAnchors[i]->weights[j]*skel->getLinearJacobian(mAnchors[i]->bodynodes[j],mAnchors[i]->local_positions[j]);//mAnchors[i]->bodynodes[j]->getTransform().inverse()*mCachedAnchorPositions[i]);
			// std::cout<<i<<std::endl;
			// std::cout<<mCachedJs[j].transpose()<<std::endl;
		}
	}
}

double
MuscleLBS::
g(double _l_m)
{
	double e_t = (l_mt -_l_m-l_t0)/l_t0;
	_l_m = _l_m/l_m0;
	double f = g_t(e_t) - (g_pl(_l_m)+activation*g_al(_l_m));
	return f;
}
double
MuscleLBS::
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
MuscleLBS::
g_pl(double _l_m)
{
	double f_pl = (exp(k_pe*(_l_m-1.0)/e_mo)-1.0)/(exp(k_pe)-1.0);
	if(_l_m<1.0)
		return 0.0;
	else
		return f_pl;
}
double
MuscleLBS::
g_al(double _l_m)
{
	return exp(-(_l_m-1.0)*(_l_m-1.0)/gamma);
}
