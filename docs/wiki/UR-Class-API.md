# UR Class API

Everything lives in the `universalRobots` namespace (`#include <ur_kinematics/ur_kinematics.h>`).

For the full generated API reference (all types, all members), see the [Doxygen docs](https://jgocunha.github.io/universal-robots-kinematics/). This page is a hand-written summary of the parts you'll actually use.

## `URtype`

```cpp
enum URtype : unsigned int { UR3, UR5, UR10 };
```

Selects which robot's Denavit-Hartenberg link dimensions (`RobotParameters`, in `robot_parameters.h`) the `UR` object uses.

## Constructing a robot

```cpp
UR(URtype robotType = UR10, bool endEffector = false, float endEffectorDimension = 0.0f);
```

- `robotType` — `UR3`, `UR5`, or `UR10`.
- `endEffector` / `endEffectorDimension` — if your robot carries a tool, `endEffectorDimension` (metres) is added as a translation past the last joint before computing the tip pose.

```cpp
universalRobots::UR robot(universalRobots::URtype::UR5);
universalRobots::UR robotWithTool(universalRobots::URtype::UR10, true, 0.15f); // tool 0.15 m past the flange
```

## `JointVector` and `IkSolutions`

```cpp
using JointVector = std::array<float, 6>; // radians, joint order 1..6

struct IkSolutions
{
    std::array<std::array<float, 6>, 8> solutions;
    std::array<bool, 8> valid;   // valid[i]: whether solutions[i] is geometrically feasible
    bool anyValid() const;       // true if at least one solution is valid
};
```

Invalid rows are filled with NaN angles rather than omitted, so `solutions` always has exactly 8 entries; check `valid[i]` (or call `isSolutionValid(solutions[i])`) before using a given row.

## `forwardKinematics`

```cpp
[[nodiscard]] pose forwardKinematics(const JointVector& targetJointVal);
```

Computes the tip `pose` for a given set of 6 joint angles (radians). Throws `std::invalid_argument` if any joint value is non-finite or outside `[-2π, 2π]`.

## `inverseKinematics`

```cpp
[[nodiscard]] IkSolutions inverseKinematics(const pose& targetTipPose);
```

Computes all 8 geometric inverse-kinematics solutions for a target tip pose (position + Euler angles). See [Kinematics Theory](Kinematics-Theory) for how the 8 solutions correspond to shoulder-left/right, elbow-up/down, and wrist-up/down configurations.

## `isPoseReachable`

```cpp
[[nodiscard]] bool isPoseReachable(const pose& targetPose);
```

Equivalent to `inverseKinematics(targetPose).anyValid()` — use this when you only need a yes/no reachability answer.

## `generateRandomReachablePose`

```cpp
pose generateRandomReachablePose();
```

Picks random joint angles within the robots' limits and runs forward kinematics, returning a pose that is reachable by construction. Useful for tests and benchmarking (see [Benchmarking](Benchmarking)).

Note that "reachable by construction" means the arm can physically get there — the pose is the forward-kinematics image of a real, in-limits joint configuration. It does **not** mean this library's analytic inverse-kinematics solver can necessarily recover a solution for it. A joint draw that swings the wrist center close to the base's vertical axis can land it inside a small cylindrical region around that axis that the solver's closed-form math cannot invert, so `isPoseReachable()` (and `inverseKinematics()`) may reject a fraction of the poses this function generates. In practice this affects roughly a fifth of draws across the full ±360° joint range (e.g. around 159/200 in a seeded UR3 run). If your use case needs every generated pose to be solvable, check `isPoseReachable()` on the result and redraw as needed.

## `isSolutionValid` (static)

```cpp
[[nodiscard]] static bool isSolutionValid(const std::array<float, 6>& ikSolution);
```

Returns `true` if none of the 6 joint values in `ikSolution` are NaN. This is what `IkSolutions::valid[i]` is derived from.

## `pose`

```cpp
struct pose
{
    std::array<float, 3> m_pos;         // x, y, z (metres)
    std::array<float, 3> m_eulerAngles; // alpha, beta, gamma (radians, ZYX convention)
};
```

Constructible from position + Euler angles, or from position + an `Eigen::Matrix3f` rotation matrix. Supports `operator-` and `operator/` for computing pose errors (see [Benchmarking](Benchmarking)).

## Printing a robot

Both `URtype` and `UR` have `operator<<` overloads for `std::ostream`, printing the robot's DH parameters, current joint values/poses, and all intermediate transformation matrices — useful for debugging.

```cpp
std::cout << robot;
```
