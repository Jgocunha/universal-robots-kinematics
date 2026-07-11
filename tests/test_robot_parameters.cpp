// test_robot_parameters.cpp — task 06: RobotParameters constants match BASELINE.md's
// table exactly. Guards against constant drift (BASELINE.md explicitly warns the
// MATLAB/CoppeliaSim UR10 model uses slightly different numbers — do not reconcile
// against those; this test pins the C++ values that are the source of truth).
#include <gtest/gtest.h>

#include <ur_kinematics/robot_parameters.h>

namespace
{
	void expectParams(const universalRobots::RobotParameters& p, universalRobots::URtype type,
					  const std::array<float, 7>& d, const std::array<float, 4>& a)
	{
		EXPECT_EQ(p.type, type);
		for (std::size_t i = 0; i < d.size(); ++i)
			EXPECT_FLOAT_EQ(p.d[i], d[i]) << "d[" << i << "]";
		for (std::size_t i = 0; i < a.size(); ++i)
			EXPECT_FLOAT_EQ(p.a[i], a[i]) << "a[" << i << "]";
	}
} // namespace

TEST(RobotParameters, UR3MatchesBaseline)
{
	expectParams(universalRobots::kUR3, universalRobots::UR3, {0.1089f, 0.1115f, 0.0f, 0.0f, 0.0007f, 0.0818f, 0.0f},
				 {0.2437f, 0.2132f, 0.0842f, 0.0011f});
}

TEST(RobotParameters, UR5MatchesBaseline)
{
	expectParams(universalRobots::kUR5, universalRobots::UR5, {0.0746f, 0.0703f, 0.0f, 0.0f, 0.0397f, 0.0829f, 0.0f},
				 {0.4251f, 0.3922f, 0.0456f, 0.0492f});
}

TEST(RobotParameters, UR10MatchesBaseline)
{
	expectParams(universalRobots::kUR10, universalRobots::UR10,
				 {0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f},
				 {0.6121f, 0.5722f, 0.0573f, 0.0584f});
}

TEST(RobotParameters, ParametersForDispatchesByType)
{
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR3), &universalRobots::kUR3);
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR5), &universalRobots::kUR5);
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR10), &universalRobots::kUR10);
}
