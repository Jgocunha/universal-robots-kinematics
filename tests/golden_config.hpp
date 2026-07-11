// golden_config.hpp — single source of truth for the golden-data matrix.
//
// Shared by the generator (which writes the golden JSON) and the characterization
// tests (which replay it), so the two never drift on model list, seed, or filenames.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <ur_kinematics/ur_kinematics.h>

namespace goldencfg
{
	struct ModelSpec
	{
		universalRobots::URtype type;
		const char* name;
	};

	// The 3 UR models covered by the golden suite.
	inline const std::vector<ModelSpec>& models()
	{
		static const std::vector<ModelSpec> m = {
			{universalRobots::URtype::UR3, "UR3"},
			{universalRobots::URtype::UR5, "UR5"},
			{universalRobots::URtype::UR10, "UR10"},
		};
		return m;
	}

	struct EeSpec
	{
		float dimension; // end-effector translation (meters), written to m_d[6]
		const char* tag; // filename tag
	};

	// FK is captured for two end-effector dimensions per model.
	inline const std::vector<EeSpec>& endEffectorDims()
	{
		static const std::vector<EeSpec> e = {
			{0.00f, "d0"},
			{0.15f, "d015"},
		};
		return e;
	}

	// Determinism knobs. Fixed forever — see tests/README.md.
	inline constexpr int kSeed = 42;
	inline constexpr int kRandomFkCount = 1000;
	inline constexpr int kIkReachableCount = 200;
	inline constexpr int kIkUnreachableCount = 10;

	// Tolerances (documented per-suite; also written into each JSON header).
	inline constexpr double kFkTolerance = 1e-5; // abs, meters / radians
	inline constexpr double kIkTolerance = 1e-4; // abs, radians per joint
	// 1e-4: cross-platform float determinism floor for the 6-DOF IK chain; matches rev1/main.
	inline constexpr double kRoundTripPosTolerance = 1e-4; // abs, meters

	inline std::string fkFileName(const std::string& model, const std::string& eeTag)
	{
		return "fk_" + model + "_" + eeTag + ".json";
	}

	inline std::string ikFileName(const std::string& model)
	{
		return "ik_" + model + ".json";
	}
} // namespace goldencfg
