// test_sanity.cpp — trivial checks that the harness links and the API is callable.
#include <gtest/gtest.h>

#include "universalRobotsKinematics.h"

TEST(Sanity, HarnessLinks)
{
	EXPECT_EQ(universalRobots::UR::m_numDoF, 6u);
	EXPECT_EQ(universalRobots::UR::m_numIkSol, 8u);
}

TEST(Sanity, ForwardKinematicsRuns)
{
	universalRobots::UR robot(universalRobots::URtype::UR5);
	const universalRobots::UR::JointVector zeros = { 0, 0, 0, 0, 0, 0 };
	const universalRobots::pose tip = robot.forwardKinematics(zeros);
	// The zero-configuration tip must be finite.
	for (float p : tip.m_pos)
		EXPECT_TRUE(std::isfinite(p));
}
