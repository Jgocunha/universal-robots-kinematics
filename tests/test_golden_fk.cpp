// test_golden_fk.cpp — replays the FK golden data and asserts numerical parity.
//
// For each model x end-effector dimension: load the golden file, run the current
// forwardKinematics() on the stored joint vector, and compare the returned tip pose
// (position + Euler angles) to the golden tip within abs tolerance kFkTolerance.
#include <gtest/gtest.h>

#include <string>

#include "universalRobotsKinematics.h"
#include "golden_config.hpp"
#include "golden_load.hpp"

namespace
{
	void checkFkFile(const goldencfg::ModelSpec& model, const goldencfg::EeSpec& ee)
	{
		const std::string fileName = goldencfg::fkFileName(model.name, ee.tag);
		const jsonmini::Value doc = goldenload::load(fileName);
		const jsonmini::Value& header = doc["header"];

		ASSERT_EQ(header["model"].asString(), model.name);
		const double tol = header["fk_tolerance"].asDouble();

		const jsonmini::Value& cases = doc["cases"];
		ASSERT_GT(cases.size(), 0u);

		universalRobots::UR robot(model.type, ee.dimension != 0.0f, ee.dimension);

		for (std::size_t c = 0; c < cases.size(); ++c)
		{
			const jsonmini::Value& tc = cases[c];
			SCOPED_TRACE(fileName + " case " + tc["name"].asString());

			universalRobots::UR::JointVector joints = {};
			const jsonmini::Value& jArr = tc["joints"];
			for (int i = 0; i < 6; ++i)
				joints[i] = jArr[i].asFloat();

			const universalRobots::pose tip = robot.forwardKinematics(joints);

			const jsonmini::Value& expected = tc["tip"];
			for (int i = 0; i < 3; ++i)
				EXPECT_NEAR(tip.m_pos[i], expected[i].asFloat(), tol) << "pos[" << i << "]";
			for (int i = 0; i < 3; ++i)
				EXPECT_NEAR(tip.m_eulerAngles[i], expected[i + 3].asFloat(), tol) << "euler[" << i << "]";
		}
	}
}

TEST(GoldenFk, AllModelsAndEndEffectors)
{
	for (const auto& model : goldencfg::models())
		for (const auto& ee : goldencfg::endEffectorDims())
			checkFkFile(model, ee);
}

// The demo fixture from Application/src/main.cpp, called out explicitly by the task.
TEST(GoldenFk, Ur5MainDemoFixture)
{
	const jsonmini::Value doc = goldenload::load(goldencfg::fkFileName("UR5", "d0"));
	const jsonmini::Value& cases = doc["cases"];

	const jsonmini::Value* demo = nullptr;
	for (std::size_t c = 0; c < cases.size(); ++c)
		if (cases[c]["name"].asString() == "main_demo")
			demo = &cases[c];
	ASSERT_NE(demo, nullptr) << "main_demo fixture missing from UR5 golden data";

	universalRobots::UR robot(universalRobots::URtype::UR5);
	universalRobots::UR::JointVector joints = {};
	for (int i = 0; i < 6; ++i)
		joints[i] = (*demo)["joints"][i].asFloat();

	const universalRobots::pose tip = robot.forwardKinematics(joints);
	const jsonmini::Value& expected = (*demo)["tip"];
	for (int i = 0; i < 3; ++i)
		EXPECT_NEAR(tip.m_pos[i], expected[i].asFloat(), goldencfg::kFkTolerance);
	for (int i = 0; i < 3; ++i)
		EXPECT_NEAR(tip.m_eulerAngles[i], expected[i + 3].asFloat(), goldencfg::kFkTolerance);
}
