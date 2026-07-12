// test_baseline_params.hpp — BASELINE.md's UR3/UR5/UR10 d/a table, shared by
// test_robot_parameters.cpp (pins RobotParameters against it) and
// test_pose_and_fk_anchors.cpp (derives the FK zero-joint anchor from it), so the
// two never drift out of sync.
#pragma once

#include <array>

namespace baselineParams
{
	struct Model
	{
		std::array<float, 7> d;
		std::array<float, 4> a;
	};

	inline constexpr Model kUR3{{0.1089f, 0.1115f, 0.0f, 0.0f, 0.0007f, 0.0818f, 0.0f},
								{0.2437f, 0.2132f, 0.0842f, 0.0011f}};
	inline constexpr Model kUR5{{0.0746f, 0.0703f, 0.0f, 0.0f, 0.0397f, 0.0829f, 0.0f},
								{0.4251f, 0.3922f, 0.0456f, 0.0492f}};
	inline constexpr Model kUR10{{0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f},
								 {0.6121f, 0.5722f, 0.0573f, 0.0584f}};
} // namespace baselineParams
