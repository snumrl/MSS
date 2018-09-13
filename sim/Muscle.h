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

	Eigen::MatrixXd GetJacobianTranspose();
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetForceJacobianAndPassive();
public:
	std::string name;
	std::vector<std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>> mAnchors;

	double f0;
	double l_mt0;

	double l_mt;
	
	double activation;
};
Eigen::Vector3d GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos);
Eigen::Vector3d GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos);

}
#endif