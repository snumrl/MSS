#ifndef __MSS_SIM_WINDOW_H__
#define __MSS_SIM_WINDOW_H__
#include "Camera.h"
#include "GLUTWindow.h"
#include "GLfunctions.h"
#include "MSSInterface.h"
#include "Environment.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <string>

namespace p = boost::python;
namespace np = boost::python::numpy;

class SimWindow : public GUI::GLUTWindow
{
public:
	SimWindow();
	SimWindow(const std::string& nn_path);
	SimWindow(const std::string& nn_path,const std::string& muscle_nn_path);
	
protected:

	/// Draw all the skeletons in mWorld. Lights and Camera are operated here.
	void Display() override;

	/// The user interactions with keyboard.
	void Keyboard(unsigned char key,int x,int y) override;

	/// Stores the data for SimWindow::Motion.
	void Mouse(int button, int state, int x, int y) override;

	/// The user interactions with mouse. Camera view is set here.
	void Motion(int x, int y) override;

	/// Reaction to window resizing.
	void Reshape(int w, int h) override;

	/// 
	void Timer(int value) override;

	/// Screenshot. The png file will be stored as ./frames/Capture/[number].png
	void Step();
	void Screenshot();
	void GetActionFromNN();
	Eigen::VectorXd GetActivationFromNN(const Eigen::VectorXd& mt);
	MSS::Environment* mWorld;

	Eigen::VectorXd mAction;
	int mActionNum;
	bool mIsRotate;
	bool mIsAuto;
	bool mIsCapture;
	bool mIsFocusing;
	bool mIsNNLoaded;
	bool mIsMuscleNNLoaded;
	bool mRandomAction;
	bool mWriteOutput;
	bool mFootControlMode;
	int mFocusBodyNum;
	int mFocusMuscle;
	double mAlpha;
	Eigen::Vector3d anchored_pos;
	//python object
	p::object mm,mns,sys_module,nn_module,muscle_nn_module,plt_module;

	int mOutputCount;
	void WriteOutput(const std::string& path);
	std::vector<std::vector<double>> plot_value;
	void Plot(const std::vector<std::vector<double>>& y);
};

#endif