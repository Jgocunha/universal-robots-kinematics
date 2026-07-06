// math_utils.h — private implementation helper for the ur_kinematics library.

#pragma once

#include <Eigen/Dense>

namespace universalRobots
{
	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} // namespace universalRobots
