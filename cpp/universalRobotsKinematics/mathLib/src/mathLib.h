// mathLib.h
#pragma once

#include <numbers>

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	float rad(const float& degree);

	float deg(const float& rad);

	struct RPY
	{
		float m_alpha = 0.0f, m_beta = 0.0f, m_gamma = 0.0f;

		RPY()
			: m_alpha(0.0f), m_beta(0.0f), m_gamma(0.0f) {}

		RPY(const float& alpha, const float& beta, const float& gamma)
			:m_alpha(alpha), m_beta(beta), m_gamma(gamma) {}

		RPY(const float(&rotationMatrix)[3][3]);

	};

	struct position
	{
		float m_x = 0.0f, m_y = 0.0f, m_z = 0.0f;

		position()
			: m_x(0.0f), m_y(0.0f), m_z(0.0f) {}

		position(const float& x, const float& y, const float& z)
			: m_x(x), m_y(y), m_z(z) {}
	};

	struct tipPose
	{
		position m_pos_e;
		RPY m_rpy_e;

		tipPose()
			: m_pos_e({ 0.0f, 0.0f, 0.0f }), m_rpy_e({ 0.0f, 0.0f, 0.0f }) {}

		tipPose(const position& pos, const RPY& rpy)
			:m_pos_e(pos), m_rpy_e(rpy) {}

	};

	struct TransformationMatrix
	{
		float m_matrix[4][4] = {{0, 0, 0, 0},
								{0, 0, 0, 0},
								{0, 0, 0, 0},
								{0, 0, 0, 0} };

		TransformationMatrix()
			:m_matrix{  {0, 0, 0, 0},
						{0, 0, 0, 0},
						{0, 0, 0, 0},
						{0, 0, 0, 0} } {}

		TransformationMatrix(const float(&matrix)[4][4])
			:m_matrix{ **matrix } {}

		friend TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2);

	};

	TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2);

	// Computes the transformation matrix from a system of coordinates to another
	float** calcTransformationMatrix(const float(&DHparams)[]);
}

