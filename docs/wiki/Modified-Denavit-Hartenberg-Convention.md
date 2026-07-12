# Modified Denavit-Hartenberg Convention

This library parameterises each UR robot using the **Modified Denavit-Hartenberg (MDH)** convention (as opposed to the "standard"/Classic DH convention). Each link transform is built from 4 parameters `{alpha_{i-1}, a_{i-1}, d_i, theta_i}`:

- `alpha_{i-1}` — twist angle between z_{i-1} and z_i, measured about x_{i-1}.
- `a_{i-1}` — link length, the distance between z_{i-1} and z_i measured along x_{i-1}.
- `d_i` — link offset, the distance between x_{i-1} and x_i measured along z_i.
- `theta_i` — joint angle, the angle between x_{i-1} and x_i measured about z_i.

This is the convention used in Craig's / Jazar's robotics textbooks, and matches the UR manufacturer parameters referenced in [`docs/REFERENCES.md`](https://github.com/Jgocunha/universal-robots-kinematics/blob/main/docs/REFERENCES.md).

## Why 9 frames for a 6-DoF robot

A UR arm has 6 revolute joints, but the last three axes (wrist) intersect at a single point and aren't mutually orthogonal in a way that a single-frame-per-joint MDH chain can express cleanly. This library splits the wrist into two extra half-frames, giving **9 reference frames** for 6 joints:

| Frame | From → To | Joint |
|---|---|---|
| 0 | `0T1` | Joint 1 |
| 1 | `1T2` | Joint 2 |
| 2 | `2T3` | Joint 3 |
| 3 | `3T4` | Joint 4 |
| 4 | `4T4'` | — (intermediate, fixed) |
| 5 | `4'T5` | Joint 5 |
| 6 | `5T5'` | — (intermediate, fixed) |
| 7 | `5'T6` | Joint 6 |
| 8 | `6T7` | — (tip / end-effector offset) |

Frames 4 and 6 (`4T4'` and `5T5'`) carry no joint angle of their own — they're fixed `alpha`/`a` rotations that re-orient the frame so the next joint's rotation axis lines up correctly. This is what lets `theta5` and `theta6` be solved independently once the wrist position is known (see [Kinematics Theory](Kinematics-Theory)).

Frame 8 (`6T7`) applies the end-effector offset (`d[6]`, set from the constructor's `endEffectorDimension` argument) and carries no joint.

## Link dimensions

Each robot's `{d_i}` (z-axis translations, 7 values) and `{a_i}` (x-axis translations, 4 values) live in `RobotParameters` (`include/ur_kinematics/robot_parameters.h`), one constant per model (`kUR3`, `kUR5`, `kUR10`). These are the measured hardware link dimensions for each UR variant; see `docs/REFERENCES.md` for the manufacturer documentation these were cross-checked against.

## Where this is built

`UR::setMDHmatrix()` (in `src/ur_kinematics.cpp`) assembles the 9×4 MDH matrix from the current joint values and the robot's `d`/`a` arrays every time joint values change. `forwardKinematics()` then converts each row into an individual transformation matrix (`calcTransformationMatrix()` in `src/math_utils.cpp`) and chains them to get the general (base-to-frame-i) transforms.
