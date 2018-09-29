#include "MSSInterface.h"
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace MSS;

void
GUI::
DrawSkeleton(
	const dart::dynamics::SkeletonPtr& skel,
	const Eigen::Vector3d& color)
{
	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

		auto T = shapeNodes.back()->getTransform();
		DrawShape(T,shapeNodes.back()->getShape().get(),color);
		if(bn->getCOMLinearVelocity().norm()>1E-5 && i==0){
			Eigen::Vector3d s,e;
			s = T.translation();
			e = s + bn->getCOMLinearVelocity();
			// GUI::DrawLine(s,e,Eigen::Vector3d(1,0,0));
		}

	}
	for(int i =0;i<skel->getNumJoints();i++)
	{
		auto parent = skel->getJoint(i)->getParentBodyNode();
		if(skel->getJoint(i)->getType()=="FreeJoint")
			continue;
		else if(skel->getJoint(i)->getType()=="BallJoint")
			glColor3f(0.8,0.2,0.2);
		else if(skel->getJoint(i)->getType()=="RevoluteJoint")
			glColor3f(0.2,0.8,0.2);
		Eigen::Isometry3d T;
		T.setIdentity();
		if(parent!=nullptr)
			T = parent->getTransform();
		T = T*skel->getJoint(i)->getTransformFromParentBodyNode();
		glPushMatrix();
		glMultMatrixd(T.data());
		
		GUI::DrawSphere(0.004);
		glPopMatrix();
    	
	}
}
void
GUI::
DrawMuscles(const std::vector<Muscle*>& muscles)
{
	glDisable(GL_LIGHTING);
	for(auto muscle : muscles)
	{
		auto aps = muscle->GetAnchors();
		double a = muscle->activation;
		// std::cout<<a<<std::endl;

		Eigen::Vector3d color(a,0.0,1.0-a);
		
		glColor3f(1.0,0.0,0.362);
		for(int i=0;i<aps.size();i++)
		{
			Eigen::Vector3d p = GetPoint(aps[i]);
			GUI::DrawSphere(p,0.006);
		}
		glColor3f(color[0],color[1],color[2]);
		for(int i=0;i<aps.size()-1;i++)
		{
			Eigen::Vector3d p = GetPoint(aps[i]);
			Eigen::Vector3d p1 = GetPoint(aps[i+1]);
			GUI::drawCylinder(0.003,p,p1);
		}
	}
	glEnable(GL_LIGHTING);
}
void
GUI::
DrawActivation(const Eigen::MatrixXd& activation)
{
	// Eigen::MatrixXi temp(activation.rows(),activation.cols());
	// for(int i =0;i<activation.rows();i++)
	// {
	// 	for(int j =0;j<activation.cols();j++)
	// 	{
	// 		temp(i,j) = activation(i,j)*256;
	// 	}	
	// }
	glDisable(GL_LIGHTING);
	int w =glutGet(GLUT_WINDOW_WIDTH);
	int h =glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0,0,w/3,w/3);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);
	glClearColor(1.0, 1.0, 1, 1);
	gluOrtho2D(0.0,1.0,0.0,1.0);
	glColor3f(1,1,1);
	glBegin(GL_QUADS);
	glVertex2f(0,0);
	glVertex2f(0,1);
	glVertex2f(1,1);
	glVertex2f(1,0);
	glEnd();
	glPointSize(10.0);
	glBegin(GL_POINTS);
	int n = activation.rows(),m = activation.cols();
	for(int i =0;i<n;i++)
	{
		for(int j =0;j<m;j++)
		{
			glColor3f(activation(i,j),activation(i,j),activation(i,j));
			glVertex2f(((float)i+0.5)/(float)n,(float)j/(float)m);
		}	
	}
	glEnd();
	glViewport(0,0,w,h);
	glEnable(GL_LIGHTING);
}
void
GUI::
DrawShape(const Eigen::Isometry3d& T,
	const dart::dynamics::Shape* shape,
	const Eigen::Vector3d& color)
{
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(color[0],color[1],color[2]);
	glPushMatrix();
	glMultMatrixd(T.data());
	if(shape->is<SphereShape>())
	{
		const auto* sphere = dynamic_cast<const SphereShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		GUI::DrawSphere(sphere->getRadius());
		// glColor3f(0,0,0);
		// glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		// GUI::DrawSphere(sphere->getRadius());
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    	GUI::DrawCube(box->getSize());
    	// GUI::DrawCube(Eigen::Vector3d(0.01,0.01,0.01));
	}
	else if(shape->is<MeshShape>())
	{
		auto* mesh = dynamic_cast<const MeshShape*>(shape);

		// for(int i =0;i<16;i++)
			// std::cout<<(*mesh->getMesh()->mRootNode->mTransformation)[i]<<" ";
    	GUI::DrawMesh(mesh->getScale(),mesh->getMesh());

	}

	glPopMatrix();

	// glDisable(GL_COLOR_MATERIAL);
}