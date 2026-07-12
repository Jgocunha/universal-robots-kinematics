// test_math_utils.cpp — task 06: unit tests for the math helpers.
//
// Covers rad/deg (round-trip + known values) and calcTransformationMatrix against
// hand-computed Modified-DH transforms (identity, pure rotations, translation-only,
// and one of the fixed frames from BASELINE.md's table). Expected matrices below are
// derived by hand from the formula documented in BASELINE.md (never by calling the
// library's forwardKinematics/inverseKinematics).
#include <gtest/gtest.h>

#include <numbers>

#include <ur_kinematics/ur_kinematics.h>
#include "math_utils.h"

namespace
{
	constexpr float kPi = std::numbers::pi_v<float>;
	constexpr float kEps = 1e-5f; // matches goldencfg::kFkTolerance

	void expectMatrixNear(const Eigen::Matrix4f& actual, const Eigen::Matrix4f& expected, float eps = kEps)
	{
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				EXPECT_NEAR(actual(r, c), expected(r, c), eps) << "at (" << r << "," << c << ")";
	}
} // namespace

// ---- rad / deg ----

TEST(MathUtils, RadKnownValues)
{
	EXPECT_NEAR(universalRobots::rad(180.0f), kPi, kEps);
	EXPECT_NEAR(universalRobots::rad(90.0f), kPi / 2.0f, kEps);
	EXPECT_NEAR(universalRobots::rad(0.0f), 0.0f, kEps);
	EXPECT_NEAR(universalRobots::rad(-90.0f), -kPi / 2.0f, kEps);
}

TEST(MathUtils, DegKnownValues)
{
	EXPECT_NEAR(universalRobots::deg(kPi), 180.0f, kEps);
	EXPECT_NEAR(universalRobots::deg(kPi / 2.0f), 90.0f, kEps);
	EXPECT_NEAR(universalRobots::deg(0.0f), 0.0f, kEps);
	EXPECT_NEAR(universalRobots::deg(-kPi / 2.0f), -90.0f, kEps);
}

TEST(MathUtils, RadDegRoundTrip)
{
	for (float degrees : {-360.0f, -123.4f, -1.0f, 0.0f, 1.0f, 45.0f, 90.0f, 180.0f, 270.0f, 359.9f})
		EXPECT_NEAR(universalRobots::deg(universalRobots::rad(degrees)), degrees, 1e-3f);
}

// ---- calcTransformationMatrix ----
// DHparams = {alpha, a, d, theta}; formula (BASELINE.md):
//   [ cosθ        -sinθ         0       a      ]
//   [ sinθ·cosα    cosθ·cosα   -sinα   -sinα·d ]
//   [ sinθ·sinα    cosθ·sinα    cosα    cosα·d ]
//   [ 0            0            0       1      ]

TEST(MathUtils, TransformIdentityAtZero)
{
	const Eigen::Matrix4f actual = universalRobots::calcTransformationMatrix({0.0f, 0.0f, 0.0f, 0.0f});
	expectMatrixNear(actual, Eigen::Matrix4f::Identity());
}

TEST(MathUtils, TransformPureRotationZ90)
{
	// alpha=0, a=0, d=0, theta=90deg -> Rz(90).
	const Eigen::Matrix4f actual = universalRobots::calcTransformationMatrix({0.0f, 0.0f, 0.0f, kPi / 2.0f});
	Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
	expected(0, 0) = 0.0f;
	expected(0, 1) = -1.0f;
	expected(1, 0) = 1.0f;
	expected(1, 1) = 0.0f;
	expectMatrixNear(actual, expected);
}

TEST(MathUtils, TransformPureRotationX90)
{
	// alpha=90deg, a=0, d=0, theta=0 -> Rx(90).
	const Eigen::Matrix4f actual = universalRobots::calcTransformationMatrix({kPi / 2.0f, 0.0f, 0.0f, 0.0f});
	Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
	expected(1, 1) = 0.0f;
	expected(1, 2) = -1.0f;
	expected(2, 1) = 1.0f;
	expected(2, 2) = 0.0f;
	expectMatrixNear(actual, expected);
}

TEST(MathUtils, TransformTranslationOnly)
{
	// alpha=0, a=2, d=3, theta=0 -> pure translation (a, 0, d).
	const Eigen::Matrix4f actual = universalRobots::calcTransformationMatrix({0.0f, 2.0f, 3.0f, 0.0f});
	Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
	expected(0, 3) = 2.0f;
	expected(2, 3) = 3.0f;
	expectMatrixNear(actual, expected);
}

TEST(MathUtils, TransformFixedFrame4T4PrimeUR5)
{
	// Row 4 of the MDH table (4T4', fixed): alpha=0, a=m_a[2], d=m_d[4], theta=+90deg.
	// UR5 (BASELINE.md table): a[2] = 0.0456, d[4] = 0.0397.
	const Eigen::Matrix4f actual = universalRobots::calcTransformationMatrix({0.0f, 0.0456f, 0.0397f, kPi / 2.0f});
	Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
	expected(0, 0) = 0.0f;
	expected(0, 1) = -1.0f;
	expected(0, 3) = 0.0456f;
	expected(1, 0) = 1.0f;
	expected(1, 1) = 0.0f;
	expected(2, 3) = 0.0397f;
	expectMatrixNear(actual, expected);
}

TEST(MathUtils, TransformFixedFrame1T2UR10)
{
	// Row 1 of the MDH table (1T2): alpha=-90deg (fixed), a=0, d=m_d[1], theta=-90deg
	// (theta2=0 at this joint value, plus the fixed -90deg offset baked into 1T2).
	// UR10 (BASELINE.md table): d[1] = 0.10122.
	const Eigen::Matrix4f actual =
		universalRobots::calcTransformationMatrix({-kPi / 2.0f, 0.0f, 0.10122f, -kPi / 2.0f});
	Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
	expected(0, 0) = 0.0f;
	expected(0, 1) = 1.0f;
	expected(1, 0) = 0.0f;
	expected(1, 1) = 0.0f;
	expected(1, 2) = 1.0f;
	expected(1, 3) = 0.10122f;
	expected(2, 0) = 1.0f;
	expected(2, 1) = 0.0f;
	expected(2, 2) = 0.0f;
	expectMatrixNear(actual, expected);
}
