// test_golden_ik.cpp — replays the IK golden data and asserts numerical parity.
//
// For each model: load the golden file and, per case, run inverseKinematics() on the
// stored target pose. Assert:
//   * every entry of the 8x6 solution matrix matches golden within kIkTolerance,
//   * NaN patterns match exactly (golden null <-> std::isnan),
//   * round-trip: each NaN-free solution row, fed back through FK, reproduces the
//     target *position* within kRoundTripPosTolerance (convention-independent
//     invariant; Euler angles excluded per quirk Q5).
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "universalRobotsKinematics.h"
#include "golden_config.hpp"
#include "golden_load.hpp"

namespace
{
	universalRobots::pose poseFromArray(const jsonmini::Value& arr)
	{
		universalRobots::pose p;
		for (int i = 0; i < 3; ++i) p.m_pos[i] = arr[i].asFloat();
		for (int i = 0; i < 3; ++i) p.m_eulerAngles[i] = arr[i + 3].asFloat();
		return p;
	}

	void checkIkFile(const goldencfg::ModelSpec& model)
	{
		const std::string fileName = goldencfg::ikFileName(model.name);
		const jsonmini::Value doc = goldenload::load(fileName);
		const jsonmini::Value& header = doc["header"];

		ASSERT_EQ(header["model"].asString(), model.name);
		const double ikTol = header["ik_tolerance"].asDouble();
		const double rtTol = header["roundtrip_pos_tolerance"].asDouble();

		const jsonmini::Value& cases = doc["cases"];
		ASSERT_GT(cases.size(), 0u);

		// IK/round-trip use the default (0.0) end-effector geometry, matching the generator.
		universalRobots::UR robot(model.type, false, 0.0f);

		for (std::size_t c = 0; c < cases.size(); ++c)
		{
			const jsonmini::Value& tc = cases[c];
			const bool reachable = tc["reachable"].asBool();
			SCOPED_TRACE(fileName + " case " + tc["name"].asString());

			const universalRobots::pose target = poseFromArray(tc["targetPose"]);

			const universalRobots::UR::IkSolutions sols = robot.inverseKinematics(target);

			const jsonmini::Value& expected = tc["solutions"];
			ASSERT_EQ(expected.size(), 8u);

			for (int s = 0; s < 8; ++s)
			{
				const jsonmini::Value& row = expected[s];
				bool rowFinite = true;
				for (int k = 0; k < 6; ++k)
				{
					const jsonmini::Value& e = row[k];
					const float actual = sols.solutions[s][k];
					if (e.isNull())
					{
						EXPECT_TRUE(std::isnan(actual))
							<< "sol " << s << " joint " << k << ": expected NaN";
						rowFinite = false;
					}
					else
					{
						EXPECT_FALSE(std::isnan(actual))
							<< "sol " << s << " joint " << k << ": unexpected NaN";
						EXPECT_NEAR(actual, e.asFloat(), ikTol)
							<< "sol " << s << " joint " << k;
						if (std::isnan(actual)) rowFinite = false;
					}
				}

				// Round-trip only for reachable targets with a fully finite solution row.
				if (reachable && rowFinite)
				{
					const universalRobots::pose fk = robot.forwardKinematics(sols.solutions[s]);
					for (int i = 0; i < 3; ++i)
						EXPECT_NEAR(fk.m_pos[i], target.m_pos[i], rtTol)
							<< "round-trip sol " << s << " pos[" << i << "]";
				}
			}
		}
	}
}

TEST(GoldenIk, AllModels)
{
	for (const auto& model : goldencfg::models())
		checkIkFile(model);
}

// Explicit assertion of quirk Q1: an unreachable target yields NaN entries.
TEST(GoldenIk, UnreachableProducesNaN)
{
	const jsonmini::Value doc = goldenload::load(goldencfg::ikFileName("UR5"));
	const jsonmini::Value& cases = doc["cases"];

	bool sawUnreachableNaN = false;
	for (std::size_t c = 0; c < cases.size(); ++c)
	{
		const jsonmini::Value& tc = cases[c];
		if (tc["reachable"].asBool()) continue;
		const jsonmini::Value& sols = tc["solutions"];
		for (int s = 0; s < 8 && !sawUnreachableNaN; ++s)
			for (int k = 0; k < 6; ++k)
				if (sols[s][k].isNull()) { sawUnreachableNaN = true; break; }
	}
	EXPECT_TRUE(sawUnreachableNaN)
		<< "expected at least one NaN entry among unreachable UR5 targets";
}
