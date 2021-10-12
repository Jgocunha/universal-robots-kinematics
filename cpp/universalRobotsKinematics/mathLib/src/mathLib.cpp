// mathLib.cpp

#include "mathLib.h"


namespace mathLib
{
	float rad(const float& degree)
	{
		return (degree * std::numbers::pi_v<float> / 180);// C++20
	   //return (degree * PI / 180); //non-compiler dependant
	}

	float deg(const float& rad)
	{
		return (rad * 180 / std::numbers::pi_v<float>);// C++20
	   //return (rad * 180 / PI); //non-compiler dependant
	}


	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams)
	{
		Eigen::Matrix4f individualTransformationMatrix;
		individualTransformationMatrix << cos(DHparams[3]), -sin(DHparams[3]), 0, DHparams[1],
			(sin(DHparams[3]) * cos(DHparams[0])), (cos(DHparams[3]) * cos(DHparams[0])), -sin(DHparams[0]), (-sin(DHparams[0]) * DHparams[2]),
			(sin(DHparams[3]) * sin(DHparams[0])), (cos(DHparams[3]) * sin(DHparams[0])), cos(DHparams[0]), (cos(DHparams[0]) * DHparams[2]),
			0, 0, 0, 1;

		return individualTransformationMatrix;
	}


} //namespace mathLib
