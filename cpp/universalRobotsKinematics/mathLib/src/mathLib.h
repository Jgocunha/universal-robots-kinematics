// mathLib.h

#pragma once

#include <numbers>
#include <Eigen/Dense>

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	struct tipPose
	{
		Eigen::Matrix<float, 1, 3> m_pos; // x y z (meters)
		Eigen::Matrix<float, 1, 3> m_rpy; // alpha beta gamma (radians)

		tipPose()
			: m_pos({ 0.0f, 0.0f, 0.0f }), m_rpy({ 0.0f, 0.0f, 0.0f }) {}

		tipPose(const Eigen::Matrix<float, 1, 3>& pos, const Eigen::Matrix<float, 1, 3>& rpy)
			:m_pos(pos), m_rpy(rpy) {}

		tipPose(const Eigen::Matrix<float, 1, 3>& pos, const Eigen::Matrix3f& rotationMatrix);

	};

	float rad(const float& degree);
	float deg(const float& rad);

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} //namespace mathLib

