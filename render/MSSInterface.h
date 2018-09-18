#ifndef __MSS_INTERFACE_H__
#define __MSS_INTERFACE_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "GLfunctions.h"
#include "Character.h"
namespace GUI
{
void DrawSkeleton(
	const dart::dynamics::SkeletonPtr& skel,
	const Eigen::Vector3d& color=Eigen::Vector3d(0.8,0.8,0.8));

void DrawMuscles(const std::vector<MSS::Muscle*>& muscles);
void DrawActivation(const Eigen::MatrixXd& activation);
void DrawShape(const Eigen::Isometry3d& T,
	const dart::dynamics::Shape* shape,
	const Eigen::Vector3d& color);
};


#endif