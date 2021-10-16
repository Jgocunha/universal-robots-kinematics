// mathLib.h

#pragma once

#include <numbers>
#include <Eigen/Dense>

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	float rad(const float& degree);
	float deg(const float& rad);

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} //namespace mathLib

