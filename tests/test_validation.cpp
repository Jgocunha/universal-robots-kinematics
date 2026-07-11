// test_validation.cpp — task 04e: FK input validation + IK valid flags.
//
// Covers:
//   * FK throws std::invalid_argument for NaN/Inf/out-of-range joint inputs
//   * isPoseReachable() / anyValid() for clearly reachable and unreachable poses
//   * Boundary IK: all-invalid solution set for a far-away pose
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <ur_kinematics/ur_kinematics.h>
#include "golden_config.hpp"

namespace
{
	constexpr float kPi   = std::numbers::pi_v<float>;
	constexpr float k2Pi  = 2.0f * kPi;

	universalRobots::UR::JointVector zeros() { return {}; }

	// Build a clearly reachable, well-conditioned pose (all-zero joints is a theta5≈0
	// wrist singularity for this robot's home configuration; using a non-singular
	// fixture here keeps these tests independent of the singularity-handling behavior
	// covered separately below).
	universalRobots::pose reachablePose()
	{
		universalRobots::UR robot;
		universalRobots::UR::JointVector joints{
			universalRobots::rad(23), universalRobots::rad(345), universalRobots::rad(78),
			universalRobots::rad(66), universalRobots::rad(77), universalRobots::rad(12) };
		return robot.forwardKinematics(joints);
	}
} // namespace

// ---- FK input validation ----

TEST(FkValidation, ThrowsOnNaN)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[2] = std::numeric_limits<float>::quiet_NaN();
	EXPECT_THROW({ (void)robot.forwardKinematics(joints); }, std::invalid_argument);
}

TEST(FkValidation, ThrowsOnPositiveInfinity)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[0] = std::numeric_limits<float>::infinity();
	EXPECT_THROW({ (void)robot.forwardKinematics(joints); }, std::invalid_argument);
}

TEST(FkValidation, ThrowsOnNegativeInfinity)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[5] = -std::numeric_limits<float>::infinity();
	EXPECT_THROW({ (void)robot.forwardKinematics(joints); }, std::invalid_argument);
}

TEST(FkValidation, ThrowsOnValueAboveTwoPi)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[1] = k2Pi + 0.001f;
	EXPECT_THROW({ (void)robot.forwardKinematics(joints); }, std::invalid_argument);
}

TEST(FkValidation, ThrowsOnValueBelowNegTwoPi)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[3] = -k2Pi - 0.001f;
	EXPECT_THROW({ (void)robot.forwardKinematics(joints); }, std::invalid_argument);
}

TEST(FkValidation, AcceptsTwoPiBoundary)
{
	universalRobots::UR robot;
	auto joints = zeros();
	joints[0] =  k2Pi;
	joints[1] = -k2Pi;
	EXPECT_NO_THROW({ (void)robot.forwardKinematics(joints); });
}

TEST(FkValidation, AcceptsZeroJoints)
{
	universalRobots::UR robot;
	EXPECT_NO_THROW({ (void)robot.forwardKinematics(zeros()); });
}

// ---- IK valid flags and isPoseReachable ----

TEST(IkValidFlags, ReachablePoseHasAtLeastOneValidSolution)
{
	universalRobots::UR robot;
	const auto sols = robot.inverseKinematics(reachablePose());
	EXPECT_TRUE(sols.anyValid());
}

TEST(IkValidFlags, UnreachablePoseHasNoValidSolution)
{
	universalRobots::UR robot;
	universalRobots::pose far;
	far.m_pos[0] = 10.0f; far.m_pos[1] = 10.0f; far.m_pos[2] = 10.0f;
	const auto sols = robot.inverseKinematics(far);
	EXPECT_FALSE(sols.anyValid());
	// Every solution row should be invalid.
	for (int s = 0; s < 8; ++s)
		EXPECT_FALSE(sols.valid[s]) << "sol " << s;
}

TEST(IkValidFlags, InvalidRowsHaveNaNAngles)
{
	universalRobots::UR robot;
	universalRobots::pose far;
	far.m_pos[0] = 10.0f; far.m_pos[1] = 10.0f; far.m_pos[2] = 10.0f;
	const auto sols = robot.inverseKinematics(far);
	for (int s = 0; s < 8; ++s)
	{
		if (sols.valid[s]) continue;
		bool hasNaN = false;
		for (int k = 0; k < 6; ++k)
			if (std::isnan(sols.solutions[s][k])) { hasNaN = true; break; }
		EXPECT_TRUE(hasNaN) << "invalid sol " << s << " should contain NaN";
	}
}

TEST(IkValidFlags, ValidRowsAreAllFinite)
{
	universalRobots::UR robot;
	const auto sols = robot.inverseKinematics(reachablePose());
	for (int s = 0; s < 8; ++s)
	{
		if (!sols.valid[s]) continue;
		for (int k = 0; k < 6; ++k)
			EXPECT_TRUE(std::isfinite(sols.solutions[s][k]))
				<< "valid sol " << s << " joint " << k << " is not finite";
	}
}

TEST(IsPoseReachable, ReturnsTrueForReachable)
{
	universalRobots::UR robot;
	EXPECT_TRUE(robot.isPoseReachable(reachablePose()));
}

TEST(IsPoseReachable, ReturnsFalseForUnreachable)
{
	universalRobots::UR robot;
	universalRobots::pose far;
	far.m_pos[0] = 10.0f; far.m_pos[1] = 10.0f; far.m_pos[2] = 10.0f;
	EXPECT_FALSE(robot.isPoseReachable(far));
}

// ---- Wrist singularity handling (task 04f) ----
//
// theta5 == 0/pi is a wrist singularity (see the convention comment in
// UR::inverseKinematics). Before task 04f these rows were unconditionally marked
// invalid; 04f lets the (already acos-clamped, task 04e) downstream math run instead,
// which stays FK-consistent even this close to the singularity.
namespace
{
	void expectValidRowsAreFkConsistent(universalRobots::UR& robot, const universalRobots::pose& target)
	{
		const auto sols = robot.inverseKinematics(target);
		EXPECT_TRUE(sols.anyValid());
		for (int s = 0; s < 8; ++s)
		{
			if (!sols.valid[s]) continue;
			for (int k = 0; k < 6; ++k)
				EXPECT_TRUE(std::isfinite(sols.solutions[s][k])) << "sol " << s << " joint " << k;

			const universalRobots::pose fk = robot.forwardKinematics(sols.solutions[s]);
			for (int i = 0; i < 3; ++i)
				EXPECT_NEAR(fk.m_pos[i], target.m_pos[i], goldencfg::kRoundTripPosTolerance)
					<< "round-trip sol " << s << " pos[" << i << "]";
		}
	}
}

TEST(WristSingularity, Theta5ZeroRowsAreValidAndFkConsistent)
{
	universalRobots::UR robot;
	const universalRobots::pose target = robot.forwardKinematics(zeros()); // theta5 == 0
	expectValidRowsAreFkConsistent(robot, target);
}

TEST(WristSingularity, Theta5PiRowsAreValidAndFkConsistent)
{
	universalRobots::UR robot;
	const universalRobots::UR::JointVector joints{ kPi, kPi, kPi, kPi, kPi, kPi }; // theta5 == pi
	const universalRobots::pose target = robot.forwardKinematics(joints);
	expectValidRowsAreFkConsistent(robot, target);
}

TEST(WristSingularity, ConventionIsDeterministic)
{
	universalRobots::UR robot;
	const universalRobots::pose target = robot.forwardKinematics(zeros());
	const auto sols1 = robot.inverseKinematics(target);
	const auto sols2 = robot.inverseKinematics(target);
	for (int s = 0; s < 8; ++s)
	{
		EXPECT_EQ(sols1.valid[s], sols2.valid[s]) << "sol " << s;
		for (int k = 0; k < 6; ++k)
		{
			const float a = sols1.solutions[s][k];
			const float b = sols2.solutions[s][k];
			if (std::isnan(a) || std::isnan(b))
				EXPECT_TRUE(std::isnan(a) && std::isnan(b)) << "sol " << s << " joint " << k;
			else
				EXPECT_EQ(a, b) << "sol " << s << " joint " << k;
		}
	}
}
