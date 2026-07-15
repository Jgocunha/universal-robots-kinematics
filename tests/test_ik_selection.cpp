// test_ik_selection.cpp — task 64: IK solution-selection helpers (joint-limit
// filter + nearest-to-current pick).
//
// All tests use a seeded std::mt19937 for determinism.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <numbers>
#include <random>
#include <stdexcept>

#include <ur_kinematics/ur_kinematics.h>
#include "golden_config.hpp"

namespace
{
	constexpr int kSeed = 6464; // distinct from other test files' seeds
	constexpr int kSweepIterationsPerModel = 300;

	universalRobots::UR::IkSolutions makeAllInvalid()
	{
		universalRobots::UR::IkSolutions sols{};
		sols.valid.fill(false);
		return sols;
	}
} // namespace

// ---- filterByJointLimits(): prune-only, never reorders, never un-invalidates ----

TEST(IkSelection, FilterMarksOutOfRangeInvalid)
{
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(true);
	sols.solutions[0] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}; // within the tight limit below
	sols.solutions[1] = {0.1f, 0.1f, 0.1f, 5.0f, 0.1f, 0.1f}; // joint 3 exceeds it

	universalRobots::JointLimits tight{};
	tight.lower.fill(-1.0f);
	tight.upper.fill(1.0f);

	universalRobots::filterByJointLimits(sols, tight);

	EXPECT_TRUE(sols.valid[0]);
	EXPECT_FALSE(sols.valid[1]);
	for (std::size_t i = 2; i < universalRobots::UR::m_numIkSol; ++i)
		EXPECT_TRUE(sols.valid[i]) << i; // default-constructed {0,...} is within the tight limit
}

TEST(IkSelection, FilterNeverFlipsAlreadyInvalidToValid)
{
	universalRobots::UR::IkSolutions sols = makeAllInvalid();
	sols.solutions[3] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}; // in-range, but valid[3] starts false

	universalRobots::filterByJointLimits(sols); // default limits

	for (bool v : sols.valid)
		EXPECT_FALSE(v);
}

TEST(IkSelection, FilterDoesNotReorderRows)
{
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(true);
	for (unsigned int i = 0; i < universalRobots::UR::m_numIkSol; ++i)
		sols.solutions[i].fill(static_cast<float>(i) * 0.01f);

	const auto before = sols.solutions;
	universalRobots::filterByJointLimits(sols); // default limits, nothing out of range
	EXPECT_EQ(sols.solutions, before);
}

TEST(IkSelection, FilterDefaultAcceptsBoundaryValues)
{
	// forwardKinematics() rejects strictly j < -2pi or j > 2pi (src/ur_kinematics.cpp), so
	// +-2pi itself must survive the default filter.
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(true);
	constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
	sols.solutions[0].fill(kTwoPi);
	sols.solutions[1].fill(-kTwoPi);

	universalRobots::filterByJointLimits(sols);

	EXPECT_TRUE(sols.valid[0]);
	EXPECT_TRUE(sols.valid[1]);
}

TEST(IkSelection, FilterExcludesAnglesForwardKinematicsWouldReject)
{
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(true);
	constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
	sols.solutions[4].fill(kTwoPi + 0.01f); // just outside FK's accepted range

	universalRobots::UR robot(universalRobots::URtype::UR5);
	EXPECT_THROW({ (void)robot.forwardKinematics(sols.solutions[4]); }, std::invalid_argument);

	universalRobots::filterByJointLimits(sols);
	EXPECT_FALSE(sols.valid[4]);
}

// ---- nearestSolution(): least-travel pick among valid rows ----

TEST(IkSelection, NearestSolutionPicksLeastTravel)
{
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(false);
	sols.valid[1] = true;
	sols.valid[5] = true;
	sols.solutions[1].fill(1.0f);
	sols.solutions[5].fill(0.2f);

	const universalRobots::UR::JointVector current{}; // all zero

	const auto result = universalRobots::nearestSolution(sols, current);
	ASSERT_TRUE(result.has_value());
	EXPECT_EQ(*result, 5u);
}

TEST(IkSelection, NearestSolutionReturnsNulloptWhenNoneValid)
{
	const universalRobots::UR::IkSolutions sols = makeAllInvalid();
	const universalRobots::UR::JointVector current{};
	EXPECT_FALSE(universalRobots::nearestSolution(sols, current).has_value());
}

TEST(IkSelection, NearestSolutionIgnoresInvalidRowsEvenIfCloser)
{
	universalRobots::UR::IkSolutions sols{};
	sols.valid.fill(false);
	sols.valid[6] = true;
	sols.solutions[2].fill(0.001f); // closer to `current`, but invalid
	sols.solutions[6].fill(2.0f);

	const universalRobots::UR::JointVector current{};
	const auto result = universalRobots::nearestSolution(sols, current);
	ASSERT_TRUE(result.has_value());
	EXPECT_EQ(*result, 6u);
}

// ---- FK-consumable guarantee: the headline correctness proof for #64 ----
//
// forwardKinematics() throws std::invalid_argument for any joint outside [-2pi, +2pi]
// (src/ur_kinematics.cpp). kDefaultUrLimits is defined to match that range exactly, so
// every solution surviving filterByJointLimits(default) is FK-consumable by
// construction; this test proves that holds across a sweep of reachable target poses,
// not just by the limits' definitional equality.
TEST(IkSelection, DefaultFilterSurvivorsAreAlwaysFkConsumable)
{
	std::mt19937 gen(kSeed);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);

	int totalChecked = 0;
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type, false, 0.0f);
		for (int iter = 0; iter < kSweepIterationsPerModel; ++iter)
		{
			universalRobots::UR::JointVector joints{};
			for (float& j : joints)
				j = dist(gen);

			const universalRobots::pose target = robot.forwardKinematics(joints);
			universalRobots::UR::IkSolutions sols = robot.inverseKinematics(target);
			universalRobots::filterByJointLimits(sols);

			for (unsigned int s = 0; s < universalRobots::UR::m_numIkSol; ++s)
			{
				if (!sols.valid[s])
					continue;
				EXPECT_NO_THROW({ (void)robot.forwardKinematics(sols.solutions[s]); })
					<< model.name << " iter " << iter << " sol " << s;
				++totalChecked;
			}
		}
	}
	EXPECT_GT(totalChecked, 0); // guards against a silent no-op regression
}
