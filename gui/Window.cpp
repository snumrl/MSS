#include "Window.h"
#include "Environment.h"
using namespace MSS;
using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;

Window::
Window()
	:mEnv(new Environment())
{
	mBackground[0] = 1.0;
	mBackground[1] = 1.0;
	mBackground[2] = 1.0;
	mBackground[3] = 1.0;
}
void
Window::
draw()
{
	for(int i=0;i<mEnv->GetWorld()->getNumSkeletons();i++)
		DrawSkeleton(mEnv->GetWorld()->getSkeleton(i));
}
void
Window::
keyboard(unsigned char _key, int _x, int _y)
{
	switch (_key)
	{
	case 's':mEnv->Step();break;
	case 27 : exit(0);break;
	default:
		break;
	}

}
void
Window::
DrawEntity(const Entity* entity) const
{
	if (!entity)
		return;
	const auto& bn = dynamic_cast<const BodyNode*>(entity);
	if(bn)
	{
		DrawBodyNode(bn);
		return;
	}

	const auto& sf = dynamic_cast<const ShapeFrame*>(entity);
	if(sf)
	{
		DrawShapeFrame(sf);
		return;
	}
}
void
Window::
DrawBodyNode(const BodyNode* bn) const
{	
	if(!bn)
		return;
	if(!mRI)
		return;

	mRI->pushMatrix();
	mRI->transform(bn->getRelativeTransform());

	auto sns = bn->getShapeNodesWith<VisualAspect>();
	for(const auto& sn : sns)
		DrawShapeFrame(sn);

	for(const auto& et : bn->getChildEntities())
		DrawEntity(et);

	mRI->popMatrix();

}
void
Window::
DrawSkeleton(const SkeletonPtr& skel) const
{
	DrawBodyNode(skel->getRootBodyNode());
}
void
Window::
DrawShapeFrame(const ShapeFrame* sf) const
{
	if(!sf)
		return;

	if(!mRI)
		return;

	const auto& va = sf->getVisualAspect();

	if(!va || va->isHidden())
		return;

	mRI->pushMatrix();
	mRI->transform(sf->getRelativeTransform());

	DrawShape(sf->getShape().get(),va->getRGBA());
	mRI->popMatrix();
}
void
Window::
DrawShape(const Shape* shape,const Eigen::Vector4d& color) const
{
	if(!shape)
		return;
	if(!mRI)
		return;

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	mRI->setPenColor(color);
	if (shape->is<SphereShape>())
	{
		const auto* sphere = static_cast<const SphereShape*>(shape);
		mRI->drawSphere(sphere->getRadius());
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = static_cast<const BoxShape*>(shape);
		mRI->drawCube(box->getSize());
	}
	else if (shape->is<CapsuleShape>())
	{
		const auto* capsule = static_cast<const CapsuleShape*>(shape);
		mRI->drawCapsule(capsule->getRadius(), capsule->getHeight());
	}
	else if (shape->is<MeshShape>())
	{
		const auto& mesh = static_cast<const MeshShape*>(shape);
		glDisable(GL_COLOR_MATERIAL);
		mRI->drawMesh(mesh->getScale(), mesh->getMesh());
	}
	glDisable(GL_COLOR_MATERIAL);
}