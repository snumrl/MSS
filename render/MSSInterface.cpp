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
		// GUI::DrawSphere(0.08);
		glPopMatrix();
    	
	}
}
void
GUI::
DrawMuscles(const std::vector<MuscleLBS*>& muscles,int focus)
{

	glDisable(GL_LIGHTING);
	int count =0;
	for(auto muscle : muscles)
	{
		// if(count==focus)
		// {


		auto aps = muscle->GetAnchors();
		double a = muscle->activation;
		Eigen::Vector3d color(10.0*a,0.0,1.0-10.0*a);
		glColor3f(1.0,0.0,0.362);
		for(int i=0;i<aps.size();i++)
		{
			Eigen::Vector3d p = aps[i]->GetPoint();
			GUI::DrawSphere(p,0.006);
		}
		// if(count==focus)
			glColor3f(color[0],color[1],color[2]);
		// else
			// glColor3f(0.0,0.0,0.0);
		for(int i=0;i<aps.size()-1;i++)
		{
			Eigen::Vector3d p = aps[i]->GetPoint();
			Eigen::Vector3d p1 = aps[i+1]->GetPoint();
			GUI::drawCylinder(0.003,p,p1);
		}
		// }
		count++;
	}
	glEnable(GL_LIGHTING);
}

void
GUI::
DrawMuscleLength(const std::vector<MSS::MuscleLBS*> muscles,int focus)
{
	glDisable(GL_LIGHTING);
	int w =glutGet(GLUT_WINDOW_WIDTH);
	int h =glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0,0,w/4,w/4);
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

	int n = muscles.size();
	double dy = 1.0/(double)n;
	
	for(int i =0;i<n;i++)
	{
		glBegin(GL_LINES);
		if(i==focus)
			glColor3f(1,0,0);
		else
			glColor3f(0,0,0);
		glVertex2f(dy*(0.5+i),0.0);
		glVertex2f(dy*(0.5+i),muscles[i]->Getl_mt()*0.5);
		glEnd();
		glColor3f(0,0,0);
		glRasterPos2f(dy*(i),0.0);
		std::string index = std::to_string(i);
		unsigned int length =index.length();
		for (unsigned int c = 0; c < length; c++)
		{
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, index.at(c) );
		}
		if(i==focus)
		{
			glRasterPos2f(0.1,0.9);
			std::string name = muscles[i]->name;
			length = name.length();
			for (unsigned int c = 0; c < length; c++)
			{
				glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, name.at(c) );
			}	
		}
	}
	glBegin(GL_LINES);
	glVertex2f(0.0,0.5);
	glVertex2f(1.0,0.5);
	glColor3f(1,0,0);
	glVertex2f(0.0,0.55);
	glVertex2f(1.0,0.55);
	glVertex2f(0.0,0.45);
	glVertex2f(1.0,0.45);
	glColor3f(0,0,1);
	glVertex2f(0.0,0.6);
	glVertex2f(1.0,0.6);
	glVertex2f(0.0,0.4);
	glVertex2f(1.0,0.4);
	glEnd();
	


	glViewport(0,0,w,h);
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