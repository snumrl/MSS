#ifndef __MSS_WINDOW_H__
#define __MSS_WINDOW_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

namespace MSS
{
class Environment;
class Muscle;
class Window : public dart::gui::Win3D
{
public:
	Window();

	void draw() override;
	void keyboard(unsigned char _key, int _x, int _y) override;
	void displayTimer(int _val) override;
private:
	void SetFocusing();

	void DrawEntity(const dart::dynamics::Entity* entity) const;
	void DrawBodyNode(const dart::dynamics::BodyNode* bn) const;
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel) const;
	void DrawShapeFrame(const dart::dynamics::ShapeFrame* shapeFrame) const;
	void DrawShape(const dart::dynamics::Shape* shape,const Eigen::Vector4d& color) const;

	void DrawMuscles(const std::vector<Muscle*>& muscles);
	void Step();
	void Reset();

	Environment* mEnv;
	bool mFocus;
	bool mSimulating;
};
};


#endif