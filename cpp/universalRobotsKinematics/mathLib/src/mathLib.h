// mathLib.h

#pragma once

#include <numbers>
#include <Eigen/Dense>

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	float rad(float degree);
	float deg(float rad);

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} //namespace mathLib

