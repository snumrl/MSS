#include "PreprocessWindow.h"
#include "dart/external/lodepng/lodepng.h"
#include "SkeletonBuilder.h"
#include "Functions.h"
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <GL/glut.h>
using namespace GUI;
using namespace dart::simulation;
using namespace dart::dynamics;
PreprocessWindow::
PreprocessWindow()
	:GLUTWindow(),mIsRotate(false),mIsCapture(false),mNumTuple(0)
{
	mWorld = new MSS::Environment(600,600);
	mDisplayTimeout = 33;

	mm = p::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = p::import("sys");
	p::str module_dir = (std::string(MSS_ROOT_DIR)+"/pymss").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);
	p::exec("import torch",mns);
	p::exec("import torch.nn as nn",mns);
	p::exec("import torch.optim as optim",mns);
	p::exec("import torch.nn.functional as F",mns);
	p::exec("import torchvision.transforms as T",mns);
	p::exec("import numpy as np",mns);
	p::exec("from MuscleNN import Regressor",mns);

	reg_module = p::eval("Regressor(1)",mns);
	reg_module.attr("GenerateTuples")(16);
}

void
PreprocessWindow::
Display() 
{
	GetFromNN();
	glClearColor(1.0, 1.0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();

	bool exist_humanoid = false;
	auto character = mWorld->GetCharacter();
	auto ground = mWorld->GetGround();
	mCamera->Apply();
	glDisable(GL_LIGHTING);
	glColor3f(0.8,0.8,0.8);
	
	float y = 
		ground->getBodyNode(0)->getTransform().translation()[1] +
		dynamic_cast<const BoxShape*>(ground->getBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	glColor3f(0,0,0);
	glLineWidth(1.0);
	Eigen::Vector3d com = character->GetSkeleton()->getCOM();
	double ox,oz;
	ox = std::fmod(com[0],0.5);
	oz = std::fmod(com[2],0.5);
	ox = com[0]-ox;
	oz = com[2]-oz;
	
	glPushMatrix();
	glTranslatef(ox,0,oz);
	glBegin(GL_LINES);
	for(float x =-20.0;x<=20.0001;x+=0.5)
	{
		glVertex3f(x,y,-20.0);
		glVertex3f(x,y,20.0);
	}
	for(float x =-20.0;x<=20.0001;x+=0.5)
	{
		glVertex3f(-20.0,y,x);
		glVertex3f(20.0,y,x);
	}
	glEnd();
	glPopMatrix();
	{
		character->GetSkeleton()->setPositions(q);
		character->GetSkeleton()->setVelocities(v);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		GUI::DrawSkeleton(character->GetSkeleton());
		int count = 0;
		for(auto m :character->GetMuscles()){
			m->activation = activation[count];
			count++;
		}
		GUI::DrawMuscles(character->GetMuscles());
	}
	{
		character->GetSkeleton()->setPositions(q_d);
		character->GetSkeleton()->setVelocities(v_d);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		GUI::DrawSkeleton(character->GetSkeleton(),Eigen::Vector3d(0.8,0.2,0.2));
	}

	

	glutSwapBuffers();
	if(mIsCapture)
		Screenshot();
	glutPostRedisplay();
}

void
PreprocessWindow::
Keyboard(unsigned char key,int x,int y) 
{
	auto character = mWorld->GetCharacter();
	switch(key)
	{
		case '`': mIsRotate= !mIsRotate;break;
		case 'C': mIsCapture = true; break;
		case 'q':std::cout<<qdd_des.transpose()<<std::endl;break;
		case 'a':std::cout<<activation.transpose()<<std::endl;break;
		case 'd':std::cout<<(A*activation+b).transpose()<<std::endl;break;
		case 'g':reg_module.attr("GenerateTuples")(16);break;
		case ']': mNumTuple=++mNumTuple;break;
		case 27 : exit(0);break;
		default : break;
	}

	glutPostRedisplay();
}
void
PreprocessWindow::
Mouse(int button, int state, int x, int y) 
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
PreprocessWindow::
Motion(int x, int y) 
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();
	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		if(!mIsRotate)
		mCamera->Translate(x,y,mPrevX,mPrevY);
		else
		mCamera->Rotate(x,y,mPrevX,mPrevY);
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		switch (mod)
		{
		case GLUT_ACTIVE_SHIFT:
			mCamera->Zoom(x,y,mPrevX,mPrevY); break;
		default:
			mCamera->Pan(x,y,mPrevX,mPrevY); break;		
		}

	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
PreprocessWindow::
Reshape(int w, int h) 
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
}
void
PreprocessWindow::
Timer(int value) 
{
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}


void PreprocessWindow::
Screenshot() {
  static int count = 0;
  const char directory[8] = "frames";
  const char fileBase[8] = "Capture";
  char fileName[32];

  boost::filesystem::create_directories(directory);
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return ;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return ;
  }
}
Eigen::VectorXd toEigenVector(const np::ndarray& array)
{
	Eigen::VectorXd vec(array.shape(0));

	float* srcs = reinterpret_cast<float*>(array.get_data());

	for(int i=0;i<array.shape(0);i++)
	{
		vec[i] = srcs[i];
	}
	return vec;
}
Eigen::MatrixXd toEigenMatrix(const np::ndarray& array)
{
	Eigen::MatrixXd mat(array.shape(0),array.shape(1));

	float* srcs = reinterpret_cast<float*>(array.get_data());

	int index = 0;
	for(int i=0;i<array.shape(0);i++)
	{
		for(int j=0;j<array.shape(1);j++)
		{
			mat(i,j) = srcs[index++];
		}
	}
	return mat;
}

void
PreprocessWindow::
GetFromNN()
{
	p::object get_tuple = reg_module.attr("GetTuple");
	p::list tuple = boost::python::extract<p::list>(get_tuple(mNumTuple));
	s = toEigenVector(np::from_object(tuple[0]));
	qdd_des = toEigenVector(np::from_object(tuple[1]));
	activation = toEigenVector(np::from_object(tuple[2]));
	A = toEigenMatrix(np::from_object(tuple[3]));
	b = toEigenVector(np::from_object(tuple[4]));
	q = toEigenVector(np::from_object(tuple[5]));
	q_d = toEigenVector(np::from_object(tuple[6]));
	v = toEigenVector(np::from_object(tuple[7]));
	v_d = toEigenVector(np::from_object(tuple[8]));
	// auto character = mWorld->GetCharacter();
	// character->GetSkeleton()->setPositions(q);
	// character->GetSkeleton()->setVelocities(v);
	// character->GetSkeleton()->computeForwardKinematics(true,false,false);
	// mWorld->GetWorld()->step();
	// q_next = character->GetSkeleton()->getPositions();
	// v_next = character->GetSkeleton()->getVelocities();
	// q_d[3] += 1.5;
	// std::cout<<"c++++"<<std::endl<<std::endl;
	// std::cout<<"s  "<<s.transpose()<<std::endl<<std::endl;
	// std::cout<<"qdd_des  "<<qdd_des.transpose()<<std::endl<<std::endl;
	// std::cout<<"activation  "<<activation.transpose()<<std::endl<<std::endl;
	// std::cout<<"A  "<<A.transpose()<<std::endl<<std::endl;
	// std::cout<<"b  "<<b.transpose()<<std::endl<<std::endl;
	// std::cout<<"q  "<<q.transpose()<<std::endl<<std::endl;
	// std::cout<<"q_d  "<<q_d.transpose()<<std::endl<<std::endl;
	// std::cout<<"v  "<<v.transpose()<<std::endl<<std::endl;
	// std::cout<<"v_d  "<<v_d.transpose()<<std::endl<<std::endl;
}
