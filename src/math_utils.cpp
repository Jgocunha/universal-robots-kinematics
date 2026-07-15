// math_utils.cpp

#include "math_utils.h"
#include <ur_kinematics/ur_kinematics.h>
#include <cmath>
#include <numbers>

namespace universalRobots
{
	float rad(float degree)
	{
		return (degree * std::numbers::pi_v<float> / 180); // C++20
														   // return (degree * PI / 180); //non-compiler dependant
	}

	float deg(float rad)
	{
		return (rad * 180 / std::numbers::pi_v<float>); // C++20
														// return (rad * 180 / PI); //non-compiler dependant
	}

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams)
	{
		Eigen::Matrix4f individualTransformationMatrix;
		individualTransformationMatrix << std::cos(DHparams[3]), -std::sin(DHparams[3]), 0, DHparams[1],
			(std::sin(DHparams[3]) * std::cos(DHparams[0])), (std::cos(DHparams[3]) * std::cos(DHparams[0])),
			-std::sin(DHparams[0]), (-std::sin(DHparams[0]) * DHparams[2]),
			(std::sin(DHparams[3]) * std::sin(DHparams[0])), (std::cos(DHparams[3]) * std::sin(DHparams[0])),
			std::cos(DHparams[0]), (std::cos(DHparams[0]) * DHparams[2]), 0, 0, 0, 1;

		return individualTransformationMatrix;
	}

} // namespace universalRobots
