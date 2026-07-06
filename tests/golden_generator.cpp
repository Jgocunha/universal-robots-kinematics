// golden_generator.cpp — produces the frozen golden JSON captured from v1.0 behavior.
//
// Run ONCE; commit the output under tests/golden/. Later tasks must not regenerate it
// except where a task brief explicitly mandates (see tests/README.md). This target is
// excluded from the default test run (it is not registered with CTest).
//
// Usage: golden_generator [output_dir]   (default: compile-time GOLDEN_DIR)
//
// Note on scope: the v1.0 public API exposes only the *tip* pose (forwardKinematics
// returns it). The 6 per-joint poses live in private state with no accessor, so they
// cannot be captured without editing the library (forbidden by task 00). We therefore
// characterize the tip pose only; see tests/README.md.

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <ur_kinematics/ur_kinematics.h>
#include "golden_config.hpp"

#ifndef GOLDEN_DIR
#define GOLDEN_DIR "."
#endif
#ifndef GIT_COMMIT
#define GIT_COMMIT "unknown"
#endif

namespace
{
	using Joints = std::array<float, 6>;

	struct NamedJoints
	{
		std::string name;
		Joints joints;
	};

	constexpr float kPi = std::numbers::pi_v<float>;

	// Format a float with full round-trip precision (9 significant digits round-trips
	// a 32-bit float exactly). NaN -> JSON null (valid JSON has no NaN literal).
	std::string f(float x)
	{
		if (std::isnan(x))
			return "null";
		char buf[32];
		std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(x));
		return buf;
	}

	std::string arr6(const float* v)
	{
		std::string s = "[";
		for (int i = 0; i < 6; ++i)
		{
			s += f(v[i]);
			if (i != 5) s += ", ";
		}
		s += "]";
		return s;
	}

	std::string poseArr(const universalRobots::pose& p)
	{
		float v[6] = { p.m_pos[0], p.m_pos[1], p.m_pos[2],
					   p.m_eulerAngles[0], p.m_eulerAngles[1], p.m_eulerAngles[2] };
		return arr6(v);
	}

	// The deterministic FK input set: hand-picked edge cases + seeded random draws.
	// Identical for every model (fixed seed, built once).
	std::vector<NamedJoints> buildFkInputs()
	{
		std::vector<NamedJoints> in;
		auto add = [&](std::string n, Joints j) { in.push_back({ std::move(n), j }); };

		// The demo fixture from Application/src/main.cpp (UR5 {23,345,78,66,77,12} deg).
		add("main_demo", { universalRobots::rad(23), universalRobots::rad(345), universalRobots::rad(78),
						   universalRobots::rad(66), universalRobots::rad(77), universalRobots::rad(12) });

		// Hand-picked edge cases.
		add("zeros", { 0, 0, 0, 0, 0, 0 });
		add("all_+pi_2", { kPi / 2, kPi / 2, kPi / 2, kPi / 2, kPi / 2, kPi / 2 });
		add("all_-pi_2", { -kPi / 2, -kPi / 2, -kPi / 2, -kPi / 2, -kPi / 2, -kPi / 2 });
		add("all_+pi", { kPi, kPi, kPi, kPi, kPi, kPi });
		add("all_-pi", { -kPi, -kPi, -kPi, -kPi, -kPi, -kPi });
		add("theta5_zero", { 0.3f, 0.3f, 0.3f, 0.3f, 0.0f, 0.3f });
		add("theta5_+1e-4", { 0.3f, 0.3f, 0.3f, 0.3f, 1e-4f, 0.3f });
		add("theta5_-1e-4", { 0.3f, 0.3f, 0.3f, 0.3f, -1e-4f, 0.3f });

		// Seeded random draws, uniform in [-2*pi, 2*pi] per joint.
		std::mt19937 gen(goldencfg::kSeed);
		std::uniform_real_distribution<float> dist(-2.0f * kPi, 2.0f * kPi);
		for (int k = 0; k < goldencfg::kRandomFkCount; ++k)
		{
			Joints j;
			for (float& x : j) x = dist(gen);
			add("random_" + std::to_string(k), j);
		}
		return in;
	}

	void writeFkFile(const std::string& outDir, const goldencfg::ModelSpec& model,
					 const goldencfg::EeSpec& ee, const std::vector<NamedJoints>& inputs)
	{
		universalRobots::UR robot(model.type, ee.dimension != 0.0f, ee.dimension);

		std::string body;
		body += "  \"cases\": [\n";
		for (std::size_t c = 0; c < inputs.size(); ++c)
		{
			const universalRobots::pose tip = robot.forwardKinematics(inputs[c].joints);

			body += "    { \"name\": \"" + inputs[c].name + "\", ";
			body += "\"joints\": " + arr6(inputs[c].joints.data()) + ", ";
			body += "\"tip\": " + poseArr(tip) + " }";
			body += (c + 1 < inputs.size()) ? ",\n" : "\n";
		}
		body += "  ]\n";

		std::string header;
		header += "{\n";
		header += "  \"header\": {\n";
		header += "    \"model\": \"" + std::string(model.name) + "\",\n";
		header += "    \"endEffectorDimension\": " + f(ee.dimension) + ",\n";
		header += "    \"seed\": " + std::to_string(goldencfg::kSeed) + ",\n";
		header += "    \"count\": " + std::to_string(inputs.size()) + ",\n";
		header += "    \"fk_tolerance\": " + std::to_string(goldencfg::kFkTolerance) + ",\n";
		header += "    \"git_commit\": \"" + std::string(GIT_COMMIT) + "\"\n";
		header += "  },\n";

		const std::string path = outDir + "/" + goldencfg::fkFileName(model.name, ee.tag);
		std::ofstream os(path, std::ios::binary);
		os << header << body << "}\n";
		std::printf("wrote %s (%zu cases)\n", path.c_str(), inputs.size());
	}

	std::string solutionsArr(const universalRobots::UR::IkSolutions& sols)
	{
		std::string s = "[";
		for (int i = 0; i < 8; ++i)
		{
			s += arr6(sols.solutions[i].data());
			if (i != 7) s += ", ";
		}
		s += "]";
		return s;
	}

	void writeIkFile(const std::string& outDir, const goldencfg::ModelSpec& model,
					 const std::vector<NamedJoints>& fkInputs)
	{
		// IK is captured at end-effector dimension 0.0 (the default model geometry).
		universalRobots::UR robot(model.type, false, 0.0f);

		std::string body;
		body += "  \"cases\": [\n";

		std::vector<std::string> records;

		// Reachable-by-construction poses: FK on the first N input joint vectors.
		const int reach = goldencfg::kIkReachableCount;
		for (int c = 0; c < reach; ++c)
		{
			const universalRobots::pose tip = robot.forwardKinematics(fkInputs[c].joints);
			const universalRobots::UR::IkSolutions sols = robot.inverseKinematics(tip);

			std::string rec;
			rec += "    { \"name\": \"reachable_" + std::to_string(c) + "\", ";
			rec += "\"reachable\": true, ";
			rec += "\"targetPose\": " + poseArr(tip) + ", ";
			rec += "\"solutions\": " + solutionsArr(sols) + " }";
			records.push_back(std::move(rec));
		}

		// Unreachable poses: positions far outside the workspace (10 m). IK produces
		// NaN in the affected solution entries (quirk Q1) — we record the pattern.
		for (int c = 0; c < goldencfg::kIkUnreachableCount; ++c)
		{
			const float base = 10.0f + static_cast<float>(c);
			universalRobots::pose far;
			far.m_pos[0] = base; far.m_pos[1] = base; far.m_pos[2] = base;
			far.m_eulerAngles[0] = 0.1f * c; far.m_eulerAngles[1] = 0.0f; far.m_eulerAngles[2] = 0.0f;
			const universalRobots::UR::IkSolutions sols = robot.inverseKinematics(far);

			std::string rec;
			rec += "    { \"name\": \"unreachable_" + std::to_string(c) + "\", ";
			rec += "\"reachable\": false, ";
			rec += "\"targetPose\": " + poseArr(far) + ", ";
			rec += "\"solutions\": " + solutionsArr(sols) + " }";
			records.push_back(std::move(rec));
		}

		for (std::size_t r = 0; r < records.size(); ++r)
		{
			body += records[r];
			body += (r + 1 < records.size()) ? ",\n" : "\n";
		}
		body += "  ]\n";

		std::string header;
		header += "{\n";
		header += "  \"header\": {\n";
		header += "    \"model\": \"" + std::string(model.name) + "\",\n";
		header += "    \"endEffectorDimension\": " + f(0.0f) + ",\n";
		header += "    \"seed\": " + std::to_string(goldencfg::kSeed) + ",\n";
		header += "    \"reachable_count\": " + std::to_string(reach) + ",\n";
		header += "    \"unreachable_count\": " + std::to_string(goldencfg::kIkUnreachableCount) + ",\n";
		header += "    \"ik_tolerance\": " + std::to_string(goldencfg::kIkTolerance) + ",\n";
		header += "    \"roundtrip_pos_tolerance\": " + std::to_string(goldencfg::kRoundTripPosTolerance) + ",\n";
		header += "    \"git_commit\": \"" + std::string(GIT_COMMIT) + "\"\n";
		header += "  },\n";

		const std::string path = outDir + "/" + goldencfg::ikFileName(model.name);
		std::ofstream os(path, std::ios::binary);
		os << header << body << "}\n";
		std::printf("wrote %s (%d reachable + %d unreachable)\n",
					path.c_str(), reach, goldencfg::kIkUnreachableCount);
	}
}

int main(int argc, char** argv)
{
	const std::string outDir = (argc > 1) ? argv[1] : GOLDEN_DIR;

	const std::vector<NamedJoints> fkInputs = buildFkInputs();

	for (const auto& model : goldencfg::models())
	{
		for (const auto& ee : goldencfg::endEffectorDims())
			writeFkFile(outDir, model, ee, fkInputs);
		writeIkFile(outDir, model, fkInputs);
	}

	std::printf("golden generation complete (git_commit %s)\n", GIT_COMMIT);
	return 0;
}
