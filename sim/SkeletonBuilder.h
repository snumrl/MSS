#ifndef __MSS_SKELETON_BUILDER_H__
#define __MSS_SKELETON_BUILDER_H__
#include "dart/dart.hpp"

namespace MSS
{
class SkeletonBuilder
{
public:
	static dart::dynamics::SkeletonPtr BuildFromFile(const std::string& filename);

	static dart::dynamics::BodyNode* MakeFreeJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact);

	static dart::dynamics::BodyNode* MakeBallJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		const Eigen::Vector3d& upper_limit,
		const Eigen::Vector3d& lower_limit,
		double mass,
		bool contact);
	
	static dart::dynamics::BodyNode* MakeRevoluteJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		double upper_limit,
		double lower_limit,
		double mass,
		const Eigen::Vector3d& axis,
		bool contact);

	static dart::dynamics::BodyNode* MakePrismaticJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		double upper_limit,
		double lower_limit,
		double mass,
		const Eigen::Vector3d& axis,
		bool contact);

	static dart::dynamics::BodyNode* MakeWeldJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact);
};
}

#endif