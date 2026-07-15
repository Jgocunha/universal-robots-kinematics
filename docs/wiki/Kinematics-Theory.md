# Kinematics Theory

This page summarises how `forwardKinematics()` and `inverseKinematics()` work. For the full derivation with figures, see the project dissertations cited in [`docs/REFERENCES.md`](https://github.com/Jgocunha/universal-robots-kinematics/blob/main/docs/REFERENCES.md).

## Forward kinematics

Given the 6 joint angles, forward kinematics is a straightforward chain of the 9 [MDH](Modified-Denavit-Hartenberg-Convention) transforms:

1. Build the 9×4 MDH parameter matrix for the current joint values (`setMDHmatrix()`).
2. Convert each row into an individual homogeneous transform `_{i-1}T_i` (`calcTransformationMatrix()`).
3. Chain them left-to-right into general transforms `_0T_i = _0T_1 \cdot _1T_2 \cdots _{i-1}T_i`.
4. `_0T_7` (frame 8) gives the tip's position and orientation directly; the position is the last column, and the orientation (Euler angles) is extracted from the rotation block via `Eigen::Matrix3f::eulerAngles(1,2,0)`, remapped to `{alpha,beta,gamma}` such that `R = RotY(gamma)·RotZ(beta)·RotX(alpha)`.

Along the way, the general transforms for the pose-bearing frames (0, 1, 2, 3, 5, 7 — corresponding to joints 1–6) are also captured as each joint's own pose, which is why `operator<<` can print every joint's position/orientation, not just the tip's.

## Inverse kinematics

Given a target tip pose, the solver **composes** a target rotation matrix from the pose's `(alpha,beta,gamma)` Euler triple as `R = RotX(alpha)·RotY(beta)·RotZ(gamma)` (note: this is a *different* convention from forward kinematics' extraction above — see [Euler convention asymmetry](#euler-convention-asymmetry-quirk-q5) below). The solver is **geometric** (closed-form), not iterative — it exploits the fact that the last three joint axes intersect at the wrist centre to decouple position from orientation, following the well-known Pieper approach for spherical-wrist manipulators (see refs. 3–5, 9 in `docs/REFERENCES.md`). It returns **up to 8 solutions**, corresponding to every combination of:

- **Shoulder left / right** (`theta1`) — two solutions for the base rotation that place the wrist centre at the required horizontal distance from the base axis.
- **Elbow up / down** (`theta3`, and consequently `theta2`, `theta4`) — two solutions for the elbow bend that reach a given wrist-centre position.
- **Wrist up / down** (`theta5`, and consequently `theta6`) — two solutions for the wrist orientation that reach the required tip orientation.

2 × 2 × 2 = 8 solutions total, indexed 0–7. `IkSolutions::valid[i]` tells you which of the 8 are geometrically feasible for the given target pose (a target outside the workspace, or requiring an unreachable combination, invalidates the corresponding rows).

### Where solutions can fail: `acos` domain checks

The solver computes several joint angles via `acos()` of a ratio derived from the target pose and the robot's link dimensions. Three sites in `inverseKinematics()` check whether that ratio falls in `[-1, 1]` (with a small epsilon for floating-point slack):

1. `theta1` (shoulder) — if out of domain, **all 8** solutions are invalid (the target is outside the reachable radius from the base).
2. `theta5` (wrist) — if out of domain for solution `i`, only that solution is invalid.
3. `theta3` (elbow) — if out of domain for solution `i`, only that solution is invalid.

### Wrist singularity

When `theta5` is 0 or ±π, the joint-6 rotation axis becomes (anti-)parallel to the joint-4 axis, and `theta6` becomes underdetermined (the usual formula divides by `sin(theta5) == 0`). By convention, this library pins `theta6 = 0` at that singularity and solves the remaining joints consistently for that choice — the wrist centre and orientation are still correct, just with one particular (arbitrary but repeatable) split between joints 4 and 6.

### Euler convention asymmetry (quirk Q5)

Forward kinematics *extracts* Euler angles from a rotation matrix as `R = RotY(gamma)·RotZ(beta)·RotX(alpha)`; inverse kinematics *composes* its target rotation matrix from the same `(alpha,beta,gamma)` triple as `R = RotX(alpha)·RotY(beta)·RotZ(gamma)`. These are two genuinely different conventions, not algebraic inverses of each other for a generic pose — not a two-fold Euler representation ambiguity. Consequently `inverseKinematics(forwardKinematics(q))` round-trips **position** exactly, but not orientation: an empirical sweep (issue #54; 500 iterations × 3 models × 8 solutions, seed 1337, 9148 valid samples) measured the rotation-invariant SO(3) geodesic distance between target and round-tripped orientation and found errors spanning the full `[0, π]` range with mean ≈1.52 rad — the statistical signature of two uncorrelated rotations, confirming a genuine convention mismatch. This is kept as documented v1.0 behavior for backward compatibility (human ruling on #54, 2026-07) rather than fixed, since fixing it would change the library's stored-Euler output contract and require a golden data re-baseline. When comparing orientations, prefer a rotation-invariant metric (e.g. the SO(3) geodesic distance, see `apps/demo/benchmarking.cpp`) over a raw per-component Euler diff.

### Getting a NaN-free solution

Use `IkSolutions::anyValid()` (or `UR::isPoseReachable()`) to check reachability, then pick a valid row with `IkSolutions::valid[i]` (or `UR::isSolutionValid(solutions[i])` on an individual row).

See the [UR Class API](UR-Class-API) page for the exact function signatures.
