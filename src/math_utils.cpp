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
		const float cos_dh0 = std::cos(DHparams[0]);
		const float sin_dh0 = std::sin(DHparams[0]);
		const float cos_dh3 = std::cos(DHparams[3]);
		const float sin_dh3 = std::sin(DHparams[3]);

		Eigen::Matrix4f individualTransformationMatrix;
		individualTransformationMatrix << cos_dh3, -sin_dh3, 0, DHparams[1], (sin_dh3 * cos_dh0), (cos_dh3 * cos_dh0),
			-sin_dh0, (-sin_dh0 * DHparams[2]), (sin_dh3 * sin_dh0), (cos_dh3 * sin_dh0), cos_dh0,
			(cos_dh0 * DHparams[2]), 0, 0, 0, 1;

		return individualTransformationMatrix;
	}

} // namespace universalRobots
