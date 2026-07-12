// test_pose_and_fk_anchors.cpp — task 06: pose operators, IkSolutions helpers, and
// FK known-pose anchors.
//
// The FK anchor expectations (all-zero joints) are derived BY HAND from the MDH
// table and calcTransformationMatrix formula in BASELINE.md, not by calling the
// library. Chaining the 9 frames' rotations/translations for theta1..6 == 0
// (frame 1 still carries its fixed -90deg offset; frames 4/4'/5/5' carry their
// fixed +90/-90deg offsets) collapses to:
//   tip.x = 0
//   tip.y = d[1] + d[2] + d[3] + d[4] + d[5] + d[6]
//   tip.z = d[0] + a[0] + a[1] + a[2] + a[3]
// (independently cross-checked with a standalone Python matrix-chain script against
// the same formula; not derived from this codebase.)
#include <gtest/gtest.h>

#include <ur_kinematics/ur_kinematics.h>

#include "test_baseline_params.hpp"

namespace
{
	constexpr float kEps = 1e-5f;
}

// ---- pose operators ----

TEST(Pose, DivideByConst)
{
	universalRobots::pose p(2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f);
	const universalRobots::pose q = p.divideByConst(2.0f);
	EXPECT_FLOAT_EQ(q.m_pos[0], 1.0f);
	EXPECT_FLOAT_EQ(q.m_pos[1], 2.0f);
	EXPECT_FLOAT_EQ(q.m_pos[2], 3.0f);
	EXPECT_FLOAT_EQ(q.m_eulerAngles[0], 4.0f);
	EXPECT_FLOAT_EQ(q.m_eulerAngles[1], 5.0f);
	EXPECT_FLOAT_EQ(q.m_eulerAngles[2], 6.0f);
}

TEST(Pose, OperatorSlashMatchesDivideByConst)
{
	universalRobots::pose p(2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f);
	const universalRobots::pose viaOperator = p / 2.0f;
	const universalRobots::pose viaMethod = p.divideByConst(2.0f);
	for (int i = 0; i < 3; ++i)
	{
		EXPECT_FLOAT_EQ(viaOperator.m_pos[i], viaMethod.m_pos[i]);
		EXPECT_FLOAT_EQ(viaOperator.m_eulerAngles[i], viaMethod.m_eulerAngles[i]);
	}
}

TEST(Pose, Subtract)
{
	universalRobots::pose a(5.0f, 7.0f, 9.0f, 1.0f, 2.0f, 3.0f);
	universalRobots::pose b(1.0f, 2.0f, 3.0f, 0.5f, 0.5f, 0.5f);
	const universalRobots::pose c = a.subtract(b);
	EXPECT_FLOAT_EQ(c.m_pos[0], 4.0f);
	EXPECT_FLOAT_EQ(c.m_pos[1], 5.0f);
	EXPECT_FLOAT_EQ(c.m_pos[2], 6.0f);
	EXPECT_FLOAT_EQ(c.m_eulerAngles[0], 0.5f);
	EXPECT_FLOAT_EQ(c.m_eulerAngles[1], 1.5f);
	EXPECT_FLOAT_EQ(c.m_eulerAngles[2], 2.5f);
}

TEST(Pose, OperatorMinusMatchesSubtract)
{
	universalRobots::pose a(5.0f, 7.0f, 9.0f, 1.0f, 2.0f, 3.0f);
	universalRobots::pose b(1.0f, 2.0f, 3.0f, 0.5f, 0.5f, 0.5f);
	const universalRobots::pose viaOperator = a - b;
	const universalRobots::pose viaMethod = a.subtract(b);
	for (int i = 0; i < 3; ++i)
	{
		EXPECT_FLOAT_EQ(viaOperator.m_pos[i], viaMethod.m_pos[i]);
		EXPECT_FLOAT_EQ(viaOperator.m_eulerAngles[i], viaMethod.m_eulerAngles[i]);
	}
}

TEST(Pose, DefaultConstructedIsZero)
{
	const universalRobots::pose p;
	for (float v : p.m_pos)
		EXPECT_FLOAT_EQ(v, 0.0f);
	for (float v : p.m_eulerAngles)
		EXPECT_FLOAT_EQ(v, 0.0f);
}

// ---- IkSolutions helpers ----

TEST(IkSolutions, AnyValidFalseWhenAllFalse)
{
	universalRobots::UR::IkSolutions sols;
	sols.valid.fill(false);
	EXPECT_FALSE(sols.anyValid());
}

TEST(IkSolutions, AnyValidTrueWhenOneTrue)
{
	universalRobots::UR::IkSolutions sols;
	sols.valid.fill(false);
	sols.valid[3] = true;
	EXPECT_TRUE(sols.anyValid());
}

TEST(IkSolutions, DefaultConstructedIsAllInvalid)
{
	universalRobots::UR::IkSolutions sols;
	EXPECT_FALSE(sols.anyValid());
	for (bool v : sols.valid)
		EXPECT_FALSE(v);
}

// ---- FK known-pose anchors: all-zero joints ----

namespace
{
	struct ModelAnchor
	{
		universalRobots::URtype type;
		std::array<float, 7> d;
		std::array<float, 4> a;
	};

	void expectZeroJointAnchor(const ModelAnchor& m)
	{
		universalRobots::UR robot(m.type, false, 0.0f);
		const universalRobots::UR::JointVector zeros = {0, 0, 0, 0, 0, 0};
		const universalRobots::pose tip = robot.forwardKinematics(zeros);

		const float expectedX = 0.0f;
		const float expectedY = m.d[1] + m.d[2] + m.d[3] + m.d[4] + m.d[5] + m.d[6];
		const float expectedZ = m.d[0] + m.a[0] + m.a[1] + m.a[2] + m.a[3];

		EXPECT_NEAR(tip.m_pos[0], expectedX, kEps);
		EXPECT_NEAR(tip.m_pos[1], expectedY, kEps);
		EXPECT_NEAR(tip.m_pos[2], expectedZ, kEps);
	}
} // namespace

TEST(FkZeroJointAnchor, UR3)
{
	expectZeroJointAnchor({universalRobots::UR3, baselineParams::kUR3.d, baselineParams::kUR3.a});
}

TEST(FkZeroJointAnchor, UR5)
{
	expectZeroJointAnchor({universalRobots::UR5, baselineParams::kUR5.d, baselineParams::kUR5.a});
}

TEST(FkZeroJointAnchor, UR10)
{
	expectZeroJointAnchor({universalRobots::UR10, baselineParams::kUR10.d, baselineParams::kUR10.a});
}
