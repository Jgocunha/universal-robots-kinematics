// test_properties.cpp — task 06: property-based tests of the kinematic invariants.
//
// All property tests use a seeded std::mt19937 (deterministic, fixed across runs);
// generateRandomReachablePose() is the one documented exception (uses random_device
// by design — only its properties are tested, not exact values).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <utility>

#include <ur_kinematics/ur_kinematics.h>
#include "golden_config.hpp"

namespace
{
	constexpr int kSeed = 1337;				  // distinct from goldencfg::kSeed; property tests own their seed.
	constexpr int kRoundTripIterations = 500; // per model; keeps total runtime well under budget.

	// A generic, non-singular reachable pose (same convention as test_validation.cpp's
	// reachablePose() fixture: hand-picked joint values away from theta5 singularities).
	universalRobots::pose genericReachablePose(universalRobots::UR& robot)
	{
		const universalRobots::UR::JointVector joints{universalRobots::rad(23), universalRobots::rad(345),
													  universalRobots::rad(78), universalRobots::rad(66),
													  universalRobots::rad(77), universalRobots::rad(12)};
		return robot.forwardKinematics(joints);
	}
} // namespace

// ---- FK . IK round trip ----

TEST(Property, FkIkRoundTrip)
{
	std::mt19937 gen(kSeed);
	std::uniform_real_distribution<float> dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);

	int totalValidChecked = 0;
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type, false, 0.0f);
		for (int iter = 0; iter < kRoundTripIterations; ++iter)
		{
			universalRobots::UR::JointVector joints{};
			for (float& j : joints)
				j = dist(gen);

			const universalRobots::pose target = robot.forwardKinematics(joints);
			const universalRobots::UR::IkSolutions sols = robot.inverseKinematics(target);

			for (int s = 0; s < 8; ++s)
			{
				if (!sols.valid[s])
					continue;
				const universalRobots::pose fk = robot.forwardKinematics(sols.solutions[s]);
				for (int i = 0; i < 3; ++i)
					EXPECT_NEAR(fk.m_pos[i], target.m_pos[i], goldencfg::kRoundTripPosTolerance)
						<< model.name << " iter " << iter << " sol " << s << " pos[" << i << "]";
				++totalValidChecked;
			}
		}
	}
	// Guards against a silent no-op regression (e.g. inverseKinematics always invalid).
	EXPECT_GT(totalValidChecked, 0);
}

// ---- generateRandomReachablePose(): non-deterministic by design, properties only ----

// Investigated as part of task 06 (see issue #42): generateRandomReachablePose()
// draws each joint uniformly in the full +-360deg range and runs FK; this does NOT
// guarantee the resulting pose is recoverable by this codebase's closed-form
// 8-solution IK solver. A wide joint draw frequently places the wrist center within
// the shoulder-offset "dead cylinder" around the base z-axis (radius less than
// d[1]+d[2]+d[3]+d[4]), which fails the solver's first acos domain check (site 1,
// theta1) and marks all 8 solutions invalid -- confirmed empirically (~20% of draws
// for UR3). This is a geometric limitation of the closed-form solver near the base
// axis, not a bug in this test or in generateRandomReachablePose() itself; documenting
// current behavior per task 06's non-goals rather than bundling a fix here.
TEST(Property, GenerateRandomReachablePoseIsUsuallyReachable)
{
	constexpr int kDraws = 30; // P(all 30 unreachable) is astronomically small at the observed ~80% success rate.
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type);
		bool sawReachable = false;
		for (int i = 0; i < kDraws; ++i)
		{
			const universalRobots::pose p = robot.generateRandomReachablePose();
			for (float v : p.m_pos)
				EXPECT_TRUE(std::isfinite(v)) << model.name << " draw " << i;
			if (robot.isPoseReachable(p))
				sawReachable = true;
		}
		EXPECT_TRUE(sawReachable) << model.name << ": expected at least one reachable draw out of " << kDraws;
	}
}

TEST(Property, GenerateRandomReachablePoseRepeatedCallsDiffer)
{
	universalRobots::UR robot(universalRobots::URtype::UR5);
	const universalRobots::pose a = robot.generateRandomReachablePose();
	const universalRobots::pose b = robot.generateRandomReachablePose();

	bool anyDiffers = false;
	for (int i = 0; i < 3; ++i)
	{
		if (a.m_pos[i] != b.m_pos[i] || a.m_eulerAngles[i] != b.m_eulerAngles[i])
			anyDiffers = true;
	}
	EXPECT_TRUE(anyDiffers) << "two consecutive draws produced an identical pose";
}

// ---- IK solution-family structure (BASELINE.md index semantics) ----
//
// Solution index i encodes: shoulder = i<4 (left) vs i>=4 (right); wrist = i in
// {0,1,4,5} (up, +acos) vs {2,3,6,7} (down, -acos); elbow = i odd vs even.
// theta1 depends only on the shoulder branch, so it is identical across all four
// indices sharing a shoulder; theta5 depends on shoulder+wrist, so it is identical
// across the two indices sharing both. These are structural invariants of the
// algorithm (BASELINE.md), independent of any specific numeric golden value.
TEST(Property, IkSolutionFamilyStructure)
{
	constexpr float kGroupEps = 1e-4f; // cross-platform float-safe: same as kIkTolerance
	for (const auto& model : goldencfg::models())
	{
		universalRobots::UR robot(model.type, false, 0.0f);
		const universalRobots::pose target = genericReachablePose(robot);
		const universalRobots::UR::IkSolutions sols = robot.inverseKinematics(target);

		ASSERT_TRUE(sols.anyValid()) << model.name;
		for (int s = 0; s < 8; ++s)
			ASSERT_TRUE(sols.valid[s]) << model.name << " sol " << s << " expected valid for a generic pose";

		// Shoulder: theta1 identical within {0,1,2,3} and within {4,5,6,7}; the two
		// groups differ (a generic pose has a non-zero shoulder-branch offset).
		for (int s : {1, 2, 3})
			EXPECT_NEAR(sols.solutions[0][0], sols.solutions[s][0], kGroupEps) << model.name << " theta1 sol " << s;
		for (int s : {5, 6, 7})
			EXPECT_NEAR(sols.solutions[4][0], sols.solutions[s][0], kGroupEps) << model.name << " theta1 sol " << s;
		EXPECT_GT(std::abs(sols.solutions[0][0] - sols.solutions[4][0]), kGroupEps)
			<< model.name << " expected the two shoulder branches to differ";

		// Wrist: theta5 identical within {0,1}, {2,3}, {4,5}, {6,7}; opposite sign
		// between the up ({0,1}/{4,5}) and down ({2,3}/{6,7}) branches of each shoulder.
		EXPECT_NEAR(sols.solutions[0][4], sols.solutions[1][4], kGroupEps) << model.name;
		EXPECT_NEAR(sols.solutions[2][4], sols.solutions[3][4], kGroupEps) << model.name;
		EXPECT_NEAR(sols.solutions[4][4], sols.solutions[5][4], kGroupEps) << model.name;
		EXPECT_NEAR(sols.solutions[6][4], sols.solutions[7][4], kGroupEps) << model.name;
		EXPECT_NEAR(sols.solutions[0][4], -sols.solutions[2][4], kGroupEps) << model.name << " wrist up/down sign";
		EXPECT_NEAR(sols.solutions[4][4], -sols.solutions[6][4], kGroupEps) << model.name << " wrist up/down sign";

		// Elbow: odd/even pairs sharing a shoulder+wrist branch take structurally
		// distinct theta3 branches (pi-psi vs pi+psi before CoppeliaSim masking); assert
		// they are not coincident rather than a brittle exact algebraic relationship,
		// which the masking step (subtract 2pi when > pi) can obscure.
		for (auto [evenIdx, oddIdx] : {std::pair{0, 1}, std::pair{2, 3}, std::pair{4, 5}, std::pair{6, 7}})
			EXPECT_GT(std::abs(sols.solutions[evenIdx][2] - sols.solutions[oddIdx][2]), kGroupEps)
				<< model.name << " expected elbow branches " << evenIdx << "/" << oddIdx << " to differ";
	}
}
