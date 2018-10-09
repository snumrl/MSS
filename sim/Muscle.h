#ifndef __MSS_MUSCLE_H__
#define __MSS_MUSCLE_H__
#include "dart/dart.hpp"

namespace MSS
{
class Muscle
{
public:
	Muscle(std::string _name,double f0,double lm0,double lt0,double pen_angle);
	void AddAnchor(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& glob_pos);
	const std::vector<std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>>& GetAnchors(){return mAnchors;}
	void Update(double time_step);
	void ApplyForceToBody();
	double GetForce();
	double Getf_A();
	double Getf_p();
	double Getl_mt();

	int GetNumRelatedDofs(){return num_related_dofs;};
	Eigen::VectorXd GetRelatedJtA();
	Eigen::MatrixXd GetJacobianTranspose();
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetForceJacobianAndPassive();
public:
	std::string name;
	std::vector<std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>> mAnchors;
	int num_related_dofs;
	std::vector<int> related_dof_indices;
public:
	//Dynamics
	double g(double _l_m);
	double g_t(double e_t);
	double g_pl(double _l_m);
	double g_al(double _l_m);
	
	double l_mt;
	double l_m;
	double activation;


	double f0;
	double l_mt0,l_m0,l_t0;

	double f_toe,e_toe,k_toe,k_lin,e_t0; //For g_t
	double k_pe,e_mo; //For g_pl
	double gamma; //For g_al
};
Eigen::Vector3d GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos);
Eigen::Vector3d GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos);

}
#endif