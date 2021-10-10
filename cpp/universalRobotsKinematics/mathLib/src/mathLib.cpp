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

	tipPose::tipPose(const Eigen::Matrix<float, 1, 3>& pos, const Eigen::Matrix3f& rotationMatrix)
		:m_pos(pos)
	{
		if (rotationMatrix(0, 2) == 1 || rotationMatrix(0, 2) == -1)
		{
			//special case
			m_rpy(0, 0) = 0; // set arbitrarily
			if (rotationMatrix(0, 2) == -1)
			{
				m_rpy(0, 1) = std::numbers::pi_v<float> / 2;
				m_rpy(0, 2) = m_rpy(0, 0) + atan2(rotationMatrix(0, 1), rotationMatrix(0, 2));
			}
			else
			{
				m_rpy(0, 1)= -std::numbers::pi_v<float> / 2;
				m_rpy(0, 2) = -m_rpy(0, 0) + atan2(rotationMatrix(0, 1), rotationMatrix(0, 2));
			}
		}
		else
		{
			m_rpy(0, 1) = -asin(rotationMatrix(0, 2)); // beta
			m_rpy(0, 2) = atan2(rotationMatrix(1, 2) / cos(m_rpy(0, 1)), rotationMatrix(2, 2) / cos(m_rpy(0, 1))); // gamma
			m_rpy(0, 0) = atan2(rotationMatrix(0, 1) / cos(m_rpy(0, 1)), rotationMatrix(0, 0) / cos(m_rpy(0, 1))); // alpha
		}
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
