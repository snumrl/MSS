#include "ContactPoint.h"
namespace MSS
{
ContactPoint::
ContactPoint(const dart::simulation::WorldPtr& world, dart::dynamics::BodyNode* bn,dart::dynamics::BodyNode* grd,const Eigen::Vector3d& v)
	:ground(grd),contact_body(bn),is_colliding(false),is_contact_on(false)
{
	local_position = contact_body->getTransform().inverse()*v;
	contact.point = Eigen::Vector3d::Zero();
	contact.normal = Eigen::Vector3d::UnitY();
	contact.force = Eigen::Vector3d::Zero();

	contact.collisionObject1 = new ContactPointCollisionObject(world,contact_body);
	contact.collisionObject2 = new ContactPointCollisionObject(world,ground);
	contact.penetrationDepth = 0.0;

	contact_constraint = std::make_shared<dart::constraint::ContactConstraint>(contact,world->getTimeStep());

	ground_y = 
		ground->getTransform().translation()[1]+
		0.5*dynamic_cast<const dart::dynamics::BoxShape*>(ground->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1];
}
Eigen::Vector3d
ContactPoint::
GetPosition()
{
	return contact_body->getTransform()*local_position;
}
void
ContactPoint::
CheckColliding()
{

	if(GetPosition()[1]<ground_y)
		is_colliding = true;
	else
		is_colliding = false;
}
void
ContactPoint::
Add(const dart::simulation::WorldPtr& world)
{
	if(is_contact_on)
		return;
	is_contact_on = true;

	contact.point = GetPosition();
	contact.penetrationDepth = (ground_y-contact.point[1]);

	contact_constraint = std::make_shared<dart::constraint::ContactConstraint>(contact,world->getTimeStep());
	world->getConstraintSolver()->addConstraint(contact_constraint);
}
void
ContactPoint::
Remove(const dart::simulation::WorldPtr& world)
{
	if(is_contact_on){
		world->getConstraintSolver()->removeConstraint(contact_constraint);
		is_contact_on =false;
	}
	contact_constraint = nullptr;
}
void
ContactPoint::
AddNeighbor(ContactPoint* other)
{
	//Check Exists
	for(int i =0;i<neighbor.size();i++)
		if(neighbor[i] == other)
			return;
	neighbor.push_back(other);
}
};