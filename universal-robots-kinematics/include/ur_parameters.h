#pragma once

#include <map>
#include <string>

#include <array>


namespace universal_robots_kinematics
{
	constexpr std::array<float, 7> ur3LinkDimensions_d = { 0.1089f, 0.1115f, 0.0f, 0.0f, 0.0007f, 0.0818f, 0.0f };
	constexpr std::array<float, 4> ur3LinkDimensions_a = { 0.2437f, 0.2132f, 0.0842f, 0.0011f };

	constexpr std::array<float, 7> ur5LinkDimensions_d = { 0.0746f, 0.0703f, 0.0f, 0.0f, 0.0397f, 0.0829f, 0.0f };
	constexpr std::array<float, 4> ur5LinkDimensions_a = { 0.4251f, 0.3922f, 0.0456f, 0.0492f };

	constexpr std::array<float, 7> ur10LinkDimensions_d = { 0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f };
	constexpr std::array<float, 4> ur10LinkDimensions_a = { 0.6121f, 0.5722f, 0.0573f, 0.0584f };

	enum RobotType : int
	{
		UR3 = 0,
		UR5 = 1,
		UR10 = 2
	};

	inline const std::map<RobotType, std::string> RobotTypeToString = {
		{UR3, "UR3" },
		{UR5, "UR5" },
		{UR10, "UR10" },
	};

	struct LinkDimensions
	{
		std::array<float, 7> d;
		std::array<float, 4> a;

		LinkDimensions(const std::array<float, 7>& d = ur10LinkDimensions_d, const std::array<float, 4>& a = ur10LinkDimensions_a)
			: d(d), a(a)
		{}
	};

	struct RobotParameters
	{
		RobotType type;
		LinkDimensions linkDimensions;
		bool endEffector; // Indicates whether a tool is/isn't attached to the robot.

		static constexpr int numDoF = 6; // Number of Degrees of Freedom is always 6 for all URs.
		static constexpr int numTransZ = 7; // Number of translations in the z-axis is always 7 (this includes a translation for the end-effector).
		static constexpr int numTransX = 4; // Number of translations in the x-axis is always 4.
		static constexpr int numReferenceFrames = 9; // Number of frames is pre-defined (according to the MDH convention it is 9).
		static constexpr int numIkSol = 8; // Number of inverse kinematics solutions for a UR is 8.

		RobotParameters(RobotType robotType = UR10, bool endEffector = false);
	};
}
