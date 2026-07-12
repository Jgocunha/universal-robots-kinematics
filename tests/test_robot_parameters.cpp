// test_robot_parameters.cpp — task 06: RobotParameters constants match BASELINE.md's
// table exactly. Guards against constant drift (BASELINE.md explicitly warns the
// MATLAB/CoppeliaSim UR10 model uses slightly different numbers — do not reconcile
// against those; this test pins the C++ values that are the source of truth).
#include <gtest/gtest.h>

#include <ur_kinematics/robot_parameters.h>

#include "test_baseline_params.hpp"

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
	expectParams(universalRobots::kUR3, universalRobots::UR3, baselineParams::kUR3.d, baselineParams::kUR3.a);
}

TEST(RobotParameters, UR5MatchesBaseline)
{
	expectParams(universalRobots::kUR5, universalRobots::UR5, baselineParams::kUR5.d, baselineParams::kUR5.a);
}

TEST(RobotParameters, UR10MatchesBaseline)
{
	expectParams(universalRobots::kUR10, universalRobots::UR10, baselineParams::kUR10.d, baselineParams::kUR10.a);
}

TEST(RobotParameters, ParametersForDispatchesByType)
{
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR3), &universalRobots::kUR3);
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR5), &universalRobots::kUR5);
	EXPECT_EQ(&universalRobots::parametersFor(universalRobots::UR10), &universalRobots::kUR10);
}
