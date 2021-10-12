// mathLib.h

#pragma once

#include <numbers>
#include <Eigen/Dense>

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	struct tipPose
	{
		float m_pos[3] = {}; // x y z (meters)
		float m_eulerAngles[3] = {}; // alpha beta gamma (radians)

		tipPose()
			: m_pos{ 0.0f, 0.0f, 0.0f }, m_eulerAngles{ 0.0f, 0.0f, 0.0f } {}

		tipPose(const float (&pos)[], const float (&eulerAngles)[])
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ eulerAngles[0], eulerAngles[1], eulerAngles[2] } {}

		tipPose(const float(&pos)[], const Eigen::Matrix3f& rotationMatrix)
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ rotationMatrix.eulerAngles(2, 1, 0).x(), rotationMatrix.eulerAngles(2, 1, 0).y(), rotationMatrix.eulerAngles(2, 1, 0).z() } {}

	};

	float rad(const float& degree);
	float deg(const float& rad);

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} //namespace mathLib

