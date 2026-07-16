# Jacobian Theory

This page summarises how `jacobian()` and `manipulability()` work: the geometric (spatial) manipulator Jacobian, its relation to the [MDH](Modified-Denavit-Hartenberg-Convention) frames this library already builds during forward kinematics, and manipulability as a measure of distance to a singularity.

## Definition

For a 6-DoF manipulator, the Jacobian `J(q)` (6×6, one column per joint) relates joint velocities to the tip's **spatial twist** — its instantaneous linear and angular velocity, stacked as a single 6-vector:

```text
x_dot = J(q) * q_dot,   x_dot = [v; omega]  (v: linear, meters/s; omega: angular, rad/s)
```

`v` and `omega` here are the tip's linear and angular velocity expressed in the **base** frame — not, in general, the time-derivative of the stored `pose::m_eulerAngles` triple (see [Analytical vs. geometric Jacobian](#analytical-vs-geometric-jacobian-why-geometric) below for why that distinction matters for this library specifically).

## Geometric Jacobian for revolute joints

All 6 UR joints are revolute. For a serial chain of revolute joints, column `i` (0-based, joint `i+1`) of the geometric Jacobian is built from two quantities, both expressed in the base frame:

- `z_i` — the unit vector along joint `i+1`'s rotation axis.
- `p_i` — the origin of the frame joint `i+1` rotates.

and the column is:

```text
J_linear[i]  = z_i x (p_end - p_i)   (top 3 rows)
J_angular[i] = z_i                    (bottom 3 rows)
```

`p_end` is the tip origin (`_0T_7`'s translation). The linear part is the classic "velocity of a point rigidly attached to a rotating frame" cross product: rotating joint `i+1` at unit rate sweeps every point rigidly downstream of it — including the tip — around the `z_i` axis through `p_i`, and a point at `p_end` moves at `z_i x (p_end - p_i)`. The angular part is simpler: rotating joint `i+1` alone contributes exactly its own axis to the tip's total angular velocity (angular velocities of a serial chain simply sum).

### Mapping columns to this library's DH frames

`forwardKinematics()` already builds `_0T_i` for all 9 [MDH frames](Modified-Denavit-Hartenberg-Convention) and caches them in `m_generalTransformationMatrices`; `jacobian()` reuses that cache directly rather than rebuilding any transforms.

The subtlety is *which* frame's z-axis/origin to use for `z_i`/`p_i`. Textbook derivations usually index by "the frame **before** joint `i+1`'s own rotation is applied" (frequently written `z_{i-1}`, `p_{i-1}`), because in general a joint's rotation changes the frame's origin and z-axis for the *next* joint's transform. But there's a simplification specific to how each joint enters this library's per-row transform (`Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta_i)`, see [MDH Convention](Modified-Denavit-Hartenberg-Convention)): `Rot_z(theta_i)` is the *last* factor, and a rotation about z leaves both the z-axis and the origin unchanged. So the frame produced by *including* joint `i+1`'s own rotation has exactly the same z-axis and origin as the frame just before that rotation — meaning `z_i`/`p_i` can be read directly off the pose-bearing frame that joint `i+1` actuates, with no extra bookkeeping:

| Joint | Column | Frame (`m_generalTransformationMatrices` index) |
|---|---|---|
| 1 | 0 | 0 (`_0T_1`) |
| 2 | 1 | 1 (`_0T_2`) |
| 3 | 2 | 2 (`_0T_3`) |
| 4 | 3 | 3 (`_0T_4`) |
| 5 | 4 | 5 (`_0T_5`, skipping intermediate frame 4 = `_0T_4'`) |
| 6 | 5 | 7 (`_0T_6`, skipping intermediate frame 6 = `_0T_5'`) |

`p_end` is frame 8 (`_0T_7`, the tip). Frames 4 and 6 are the fixed, non-joint-bearing half-frames the wrist needs (see the MDH page); they never supply a Jacobian column.

## Manipulability

The Yoshikawa manipulability index measures how far a configuration `q` is from a singularity — a configuration where `J(q)` loses rank and some tip velocity directions become unreachable no matter how fast the joints move:

```text
w(q) = sqrt(det(J(q) * J(q)^T))
```

For this library's Jacobian, which is always square (6x6, one column per joint, no redundancy), `det(J * J^T) = det(J)^2`, so `sqrt(det(J * J^T)) == abs(det(J))`. `manipulability()` computes it that way — `abs(J(q).determinant())` — which is both faster (no 6x6 matrix product) and sidesteps the float-noise-negative-radicand problem a literal `sqrt(det(J*J^T))` would have right at a singularity, rather than computing then clamping it: `abs()` of a finite `float` is always finite and non-negative, so the result is never `NaN` and is `0.0f` (or vanishingly close) exactly at a singularity.

`w(q)` is a single scalar summary; the **smallest singular value** of `J(q)` (e.g. via `Eigen::JacobiSVD`) is a related, more granular measure of the same thing — it goes to 0 exactly where `w(q)` does, but also identifies *which* direction in velocity space is becoming unreachable (the corresponding right-singular vector), which `w(q)` alone does not.

Both are **continuous** distance-to-singularity measures: they shrink smoothly to 0 as `q` approaches a singularity, from any direction. This is a different kind of tool than the small `constexpr` epsilons already in `inverseKinematics()`'s `solveTheta6()` (`kZeroSingularityEps`, `kPiSingularityEps` in `src/ur_kinematics.cpp`) — those are **binary** guards that pin a *specific downstream formula* (an `atan2` that divides by `sin(theta5)`) to a fixed fallback value once `theta5` gets numerically close enough to 0 or ±π that the division would blow up. They answer "is this one division about to be unsafe?", not "how manipulable is this configuration overall?" — `jacobian()`/`manipulability()` are a general-purpose tool usable at any `q`, independent of `inverseKinematics()` and its specific solver internals.

## The three UR singularity families

A UR's rank-6 Jacobian drops rank at three configuration families (using this library's joint numbering, 1-indexed, radians):

1. **Wrist singularity** — `theta5 = 0` (or `±π`). Joint 6's axis becomes (anti-)parallel to joint 4's axis, so the wrist loses one rotational degree of freedom (two joints now contribute the same rotation direction). This is the same condition `solveTheta6()`'s `kZeroSingularityEps`/`kPiSingularityEps` guard against on the IK side, from the opposite direction — there, an underdetermined `theta6` at the singularity; here, a rank-deficient `J`.
2. **Elbow singularity** — `theta3 = 0` or `theta3 = π`. The arm is fully extended or fully folded; the elbow can no longer move the wrist centre perpendicular to the (shoulder → wrist-centre) line, collapsing one linear degree of freedom.
3. **Shoulder singularity** — the wrist centre lies on the joint-1 (base) rotation axis. Rotating the base contributes no tip velocity component in the direction toward/away from that axis, since the wrist centre is sitting on it.

At each, `rank(J(q)) < 6`, `w(q) -> 0`, and the smallest singular value of `J(q) -> 0` — empirically (see `tests/test_jacobian.cpp`), both measure on the order of `1e-5`-`1e-8` at an exact wrist or elbow singularity, several orders of magnitude below a generic configuration.

## Analytical vs. geometric Jacobian: why geometric

A manipulator Jacobian can be built two ways:

- **Geometric (spatial)** — the one this library implements. Columns are `[z_i x (p_end - p_i); z_i]`, and the angular rows relate joint rates to true angular velocity `omega`. Convention-free: it doesn't depend on any particular choice of orientation representation.
- **Analytical** — relates joint rates to the *time-derivative of a specific orientation parameterisation* (e.g. Euler angles), `x_dot_analytical = [v; d(euler)/dt]`. This requires an extra transformation (an Euler-rate mapping matrix, itself a function of the current Euler angles and singular at certain orientations of its own — gimbal lock in the representation, on top of any physical singularity) and is tied to *which* Euler convention you pick.

This library picked geometric deliberately because of **Quirk Q5** (see [Kinematics Theory](Kinematics-Theory#euler-convention-asymmetry-quirk-q5)): `forwardKinematics()` *extracts* `pose::m_eulerAngles` as `R = RotY(gamma)*RotZ(beta)*RotX(alpha)`, while `inverseKinematics()` *composes* its target rotation from the same triple as `R = RotX(alpha)*RotY(beta)*RotZ(gamma)` — two different, non-inverse conventions for the same stored triple. An analytical Jacobian would have to commit to *one* of those conventions (or a third), and differentiating either through its convention's singularities would add a second, representation-only singularity family layered on top of the three genuine physical ones above — with no way to pick one that's obviously "correct" given the library's existing FK/IK asymmetry. The geometric Jacobian sidesteps this entirely: `z_i` and `p_i` come directly from the same rotation/translation blocks `forwardKinematics()` already computes, with no Euler angles involved anywhere in the derivation, so it has no relationship to (and is not affected by) Quirk Q5 at all.

This is also why the validation tests (`tests/test_jacobian.cpp`) check the Jacobian's **linear** rows against a finite difference of tip *position* (convention-free), but validate the **angular** rows structurally (each column's angular part is a unit vector, since it's defined to equal a rotation axis) rather than by finite-differencing the stored Euler angles — differentiating through the FK-extract convention would silently test that convention's Euler-rate behavior, not the geometric angular velocity `jacobian()` actually returns.

See the [UR Class API](UR-Class-API) page for the exact `jacobian()`/`manipulability()` signatures.
