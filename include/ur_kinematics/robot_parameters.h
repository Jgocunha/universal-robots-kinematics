// robot_parameters.h

#pragma once

#include <array>

namespace universalRobots
{

	/// <summary> URtype
	/// Different types of URs.
	/// </summary>
	enum URtype : unsigned int
	{
		UR3,
		UR5,
		UR10 // 0, 1, 2
	};

	/// <summary>
	/// Denavit-Hartenberg link dimensions (meters) for a UR model.
	/// d - z-axis translations (d[6] is the end-effector slot); a - x-axis translations.
	/// </summary>
	struct RobotParameters
	{
		URtype type;
		std::array<float, 7> d;
		std::array<float, 4> a;
	};

	inline constexpr RobotParameters kUR3{
		UR3, {0.1089f, 0.1115f, 0.0f, 0.0f, 0.0007f, 0.0818f, 0.0f}, {0.2437f, 0.2132f, 0.0842f, 0.0011f}};
	inline constexpr RobotParameters kUR5{
		UR5, {0.0746f, 0.0703f, 0.0f, 0.0f, 0.0397f, 0.0829f, 0.0f}, {0.4251f, 0.3922f, 0.0456f, 0.0492f}};
	inline constexpr RobotParameters kUR10{
		UR10, {0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f}, {0.6121f, 0.5722f, 0.0573f, 0.0584f}};

	/// <summary>
	/// Returns the DH parameters for a given UR model. Unknown values fall back to UR10.
	/// </summary>
	constexpr const RobotParameters& parametersFor(const URtype type)
	{
		switch (type)
		{
		case UR3:
			return kUR3;
		case UR5:
			return kUR5;
		case UR10:
			return kUR10;
		default:
			return kUR10;
		}
	}

} // namespace universalRobots
