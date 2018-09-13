#ifndef __MSS_CONTACT_POINT_H__
#define __MSS_CONTACT_POINT_H__
#include "dart/dart.hpp"

namespace MSS
{
class ContactPoint
{
public:
	ContactPoint(const dart::simulation::WorldPtr& world, dart::dynamics::BodyNode* bn,dart::dynamics::BodyNode* grd,const Eigen::Vector3d& v);
	Eigen::Vector3d GetPosition();
	
	void CheckColliding();
	void Add(const dart::simulation::WorldPtr& world);
	void Remove(const dart::simulation::WorldPtr& world);
	void AddNeighbor(ContactPoint* other);

	const std::vector<ContactPoint*>& GetNeighbor(){return neighbor;};
	bool IsColliding(){return is_colliding;}
	bool IsContactOn(){return is_contact_on;}
private:
	dart::dynamics::BodyNode* ground;
	dart::dynamics::BodyNode* contact_body;
	Eigen::Vector3d local_position;

	dart::collision::Contact contact;
	dart::constraint::ContactConstraintPtr	contact_constraint;

	std::vector<ContactPoint*> neighbor;
	double ground_y;
	bool is_colliding;
	bool is_contact_on;
};

class ContactPointCollisionObject: public dart::collision::CollisionObject
{
public:
	ContactPointCollisionObject(const dart::simulation::WorldPtr& world,dart::dynamics::BodyNode* bn):CollisionObject(world->getConstraintSolver()->getCollisionDetector().get(),dynamic_cast<dart::dynamics::ShapeFrame*>(bn->getShapeNodesWith<dart::dynamics::VisualAspect>()[0])){};
protected:
	virtual void updateEngineData(){};
};
};
#endif