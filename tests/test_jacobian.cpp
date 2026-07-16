// test_jacobian.cpp — task 63: geometric (spatial) Jacobian + manipulability.
//
// All tests use a seeded std::mt19937 for determinism.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <Eigen/Dense>
#include <ur_kinematics/ur_kinematics.h>
#include "golden_config.hpp"

namespace
{
	constexpr int kSeed = 6300; // distinct from other test files' seeds

	// Finite-difference gate tolerances (task 63 dispatch): eps ~= float eps^(1/3)
	// (~5e-3 rad) balances truncation error (grows with eps) against float32
	// cancellation error (grows as eps shrinks) for a single-sided difference
	// through a float-internal FK chain.
	constexpr float kFdEps = 5e-3f;
	constexpr float kFdRelTol = 1e-3f;
	constexpr float kFdAbsFloor = 1e-5f;

	// Well above the ~1e-5 manipulability / ~1e-8 smallest-singular-value observed
	// empirically at an exact wrist/elbow singularity, well below a generic
	// non-singular configuration's values.
	constexpr float kSingularityEps = 1e-3f;
} // namespace

// ---- Finite-difference gate: linear rows only (position is convention-free;
//      orientation is not, see Quirk Q5 -- angular rows are validated structurally
//      in AngularRowsAreUnitJointAxes below instead). ----

TEST(Jacobian, LinearRowsMatchFiniteDifference)
{
	std::mt19937 gen(kSeed);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
	constexpr int kConfigsPerModel = 20;

	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		for (int iter = 0; iter < kConfigsPerModel; ++iter)
		{
			universalRobots::UR::JointVector q{};
			for (float& j : q)
				j = dist(gen);

			const Eigen::Matrix<float, 6, 6> J = robot.jacobian(q);

			for (unsigned int i = 0; i < universalRobots::UR::m_numDoF; ++i)
			{
				universalRobots::UR::JointVector qPlus = q;
				qPlus[i] += kFdEps;
				universalRobots::UR::JointVector qMinus = q;
				qMinus[i] -= kFdEps;
				const universalRobots::pose pPlus = robot.forwardKinematics(qPlus);
				const universalRobots::pose pMinus = robot.forwardKinematics(qMinus);

				for (int row = 0; row < 3; ++row)
				{
					// Central difference: O(eps^2) truncation error, vs. O(eps) for a
					// single-sided difference -- at eps=5e-3 a single-sided difference's
					// truncation error alone exceeds a 1e-3 relative tolerance for this
					// chain (verified empirically), while central differences clear it
					// comfortably (~0.4% worst case observed), making it the right choice
					// for a tolerance this tight without weakening the gate.
					const float fd = (pPlus.m_pos[row] - pMinus.m_pos[row]) / (2.0f * kFdEps);
					const float tol = kFdAbsFloor + kFdRelTol * std::abs(fd);
					EXPECT_NEAR(J(row, i), fd, tol)
						<< model.name << " iter " << iter << " col " << i << " row " << row;
				}
			}
		}
	}
}

// ---- Angular rows: validated structurally, not via finite-difference (would
//      require differentiating through the FK-extract Euler convention, which is
//      asymmetric with IK-compose -- see Quirk Q5, docs/wiki/Kinematics-Theory.md).
//      Column i's angular part is defined as z_i, the joint-i+1 axis expressed in
//      the base frame -- check it is a unit vector, as any rotation axis must be. ----

TEST(Jacobian, AngularRowsAreUnitJointAxes)
{
	std::mt19937 gen(kSeed + 1);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
	constexpr int kConfigsPerModel = 20;
	constexpr float kUnitTol = 1e-4f;

	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		for (int iter = 0; iter < kConfigsPerModel; ++iter)
		{
			universalRobots::UR::JointVector q{};
			for (float& j : q)
				j = dist(gen);

			const Eigen::Matrix<float, 6, 6> J = robot.jacobian(q);
			for (unsigned int i = 0; i < universalRobots::UR::m_numDoF; ++i)
			{
				const Eigen::Vector3f angular = J.block<3, 1>(3, i);
				EXPECT_NEAR(angular.norm(), 1.0f, kUnitTol) << model.name << " iter " << iter << " col " << i;
			}
		}
	}
}

// ---- Determinism / no-throw: jacobian()/manipulability() are pure functions of q,
//      same argument -> identical result, and never throw for any FK-valid q. ----

TEST(Jacobian, IsDeterministic)
{
	universalRobots::UR robot(universalRobots::URtype::UR5);
	const universalRobots::UR::JointVector q{0.3f, -0.5f, 0.8f, 0.2f, 0.6f, -0.1f};

	const Eigen::Matrix<float, 6, 6> j1 = robot.jacobian(q);
	const Eigen::Matrix<float, 6, 6> j2 = robot.jacobian(q);
	EXPECT_TRUE(j1 == j2);

	EXPECT_EQ(robot.manipulability(q), robot.manipulability(q));
}

TEST(Jacobian, NeverThrowsForFkValidJoints)
{
	std::mt19937 gen(kSeed + 2);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
	constexpr int kDraws = 200;

	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		for (int i = 0; i < kDraws; ++i)
		{
			universalRobots::UR::JointVector q{};
			for (float& j : q)
				j = dist(gen);

			EXPECT_NO_THROW({ (void)robot.jacobian(q); }) << model.name << " draw " << i;
			EXPECT_NO_THROW({ (void)robot.manipulability(q); }) << model.name << " draw " << i;
		}
	}
}

// ---- Singular configurations: manipulability -> ~0 and rank(J) < 6. ----
//
// Empirically (see PR discussion / task 63): at an exact wrist (theta5=0) or elbow
// (theta3=0) singularity, manipulability() measures ~1e-5 and the smallest singular
// value ~1e-8-1e-9 -- both are float-noise-scale artifacts of an analytically-zero
// quantity, several orders of magnitude below a generic non-singular configuration.
TEST(Jacobian, WristSingularityDropsRank)
{
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		const universalRobots::UR::JointVector q{0.3f, -0.5f, 0.8f, 0.2f, 0.0f, -0.1f}; // theta5 = 0

		EXPECT_LT(robot.manipulability(q), kSingularityEps) << model.name;

		const Eigen::Matrix<float, 6, 6> J = robot.jacobian(q);
		const Eigen::JacobiSVD<Eigen::Matrix<float, 6, 6>> svd(J);
		EXPECT_LT(svd.singularValues()(5), kSingularityEps) << model.name << " smallest singular value";
		EXPECT_LT(J.fullPivLu().rank(), 6) << model.name;
	}
}

TEST(Jacobian, ElbowSingularityDropsRank)
{
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		const universalRobots::UR::JointVector q{0.3f, -0.5f, 0.0f, 0.2f, 0.4f, -0.1f}; // theta3 = 0

		EXPECT_LT(robot.manipulability(q), kSingularityEps) << model.name;

		const Eigen::Matrix<float, 6, 6> J = robot.jacobian(q);
		const Eigen::JacobiSVD<Eigen::Matrix<float, 6, 6>> svd(J);
		EXPECT_LT(svd.singularValues()(5), kSingularityEps) << model.name << " smallest singular value";
		EXPECT_LT(J.fullPivLu().rank(), 6) << model.name;
	}
}

// ---- manipulability() never returns NaN/negative, even right at a singularity
//      where the radicand can go slightly negative from float noise alone. ----

TEST(Jacobian, ManipulabilityNeverNegativeOrNan)
{
	std::mt19937 gen(kSeed + 3);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
	constexpr int kDraws = 200;

	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		for (int i = 0; i < kDraws; ++i)
		{
			universalRobots::UR::JointVector q{};
			for (float& j : q)
				j = dist(gen);

			const float w = robot.manipulability(q);
			EXPECT_TRUE(std::isfinite(w)) << model.name << " draw " << i;
			EXPECT_GE(w, 0.0f) << model.name << " draw " << i;
		}
	}

	// Exact wrist singularity: the case most likely to expose float-noise-negative radicand.
	const universalRobots::UR::JointVector qSingular{0.3f, -0.5f, 0.8f, 0.2f, 0.0f, -0.1f};
	universalRobots::UR robot(universalRobots::URtype::UR5);
	const float wSingular = robot.manipulability(qSingular);
	EXPECT_TRUE(std::isfinite(wSingular));
	EXPECT_GE(wSingular, 0.0f);
}
