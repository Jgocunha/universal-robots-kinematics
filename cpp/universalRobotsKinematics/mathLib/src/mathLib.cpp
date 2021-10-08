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

	RPY::RPY(const float(&rotationMatrix)[3][3])
	{
		if (rotationMatrix[0][2] == 1 || rotationMatrix[0][2] == -1)
		{
			//special case
			m_alpha = 0; // set arbitrarily
			if (rotationMatrix[0][2] == -1)
			{
				m_beta = std::numbers::pi_v<float> / 2;
				m_gamma = m_alpha + atan2(rotationMatrix[0][1], rotationMatrix[0][2]);
			}
			else
			{
				m_beta = -std::numbers::pi_v<float> / 2;
				m_gamma = -m_alpha + atan2(rotationMatrix[0][1], rotationMatrix[0][2]);
			}
		}
		else
		{
			m_beta = -asin(rotationMatrix[0][2]);
			m_gamma = atan2(rotationMatrix[1][2] / cos(m_beta), rotationMatrix[2][2] / cos(m_beta));
			m_alpha = atan2(rotationMatrix[0][1] / cos(m_beta), rotationMatrix[0][0] / cos(m_beta));
		}
	}

	float** calcTransformationMatrix(const float(&DHparams)[])
	{
		//static 
		float individualTransformationMatrix[4][4] = {  {cos(DHparams[3]),								-sin(DHparams[3]),							0,							DHparams[1]},
														{(sin(DHparams[3]) * cos(DHparams[0])),		(cos(DHparams[3]) * cos(DHparams[0])),	    -sin(DHparams[0]),	    (-sin(DHparams[0]) * DHparams[2])},
														{(sin(DHparams[3]) * sin(DHparams[0])),		(cos(DHparams[3]) * sin(DHparams[0])),	    cos(DHparams[0]),		(cos(DHparams[0]) * DHparams[2])},
														{0,														0,										0,							1} };
		return reinterpret_cast<float**>(individualTransformationMatrix);
	}

	TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2)
	{
		//static 
		float result[4][4] = {};

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				result[i][j] = 0;
				for (int k = 0; k < 4; k++) {
					result[i][j] += mat1.m_matrix[i][k] * mat2.m_matrix[k][j];
				}
			}
		}

		return reinterpret_cast<TransformationMatrix**>(result);
	}
}
