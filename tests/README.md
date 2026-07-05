# Golden characterization suite

This directory is the **numerical safety net** for the modernization of
`universal-robots-kinematics`. It captures the exact behavior of the v1.0 library
(commit recorded in each golden file's `git_commit` header) as machine-readable
**golden data**, plus a GoogleTest suite that replays it. As long as this suite stays
green, no observable kinematics behavior was lost in a refactor.

Introduced by roadmap **task 00**. It adds **zero diffs under `cpp/`** — the library is
consumed through its public API only.

## Layout

| File | Purpose |
|---|---|
| `golden/*.json` | Frozen reference data generated from v1.0. **Do not edit or regenerate** (see below). |
| `golden_generator.cpp` | One-shot executable (target `golden_generator`) that produces `golden/*.json`. Not run by CTest. |
| `test_golden_fk.cpp` | FK characterization tests. |
| `test_golden_ik.cpp` | IK + round-trip characterization tests. |
| `test_sanity.cpp` | Trivial link/callable checks. |
| `golden_config.hpp` | Single source of truth for the model matrix, seed, counts, tolerances, filenames. |
| `json_mini.hpp` | Tiny dependency-free JSON reader (tests only; the library never depends on it). |
| `golden_load.hpp` | Reads a golden file from the compiled-in golden dir. |

## What is captured

**Forward kinematics** — `fk_<MODEL>_<eeTag>.json` for UR3/UR5/UR10 ×
end-effector dimension ∈ {`d0` = 0.0 m, `d015` = 0.15 m}. Each file holds 1009 cases:
- the `main_demo` fixture (UR5 `{23,345,78,66,77,12}`° from `Application/src/main.cpp`),
- hand-picked edge cases: all-zeros, all ±π/2, all ±π, and θ5 ∈ {0, ±1e-4} (near the
  wrist singularity, quirk Q3),
- 1000 random joint vectors from `std::mt19937` seed **42**, uniform in [−2π, 2π] per joint.

Each case records the input `joints` and the resulting **tip** `pose`
(`[x, y, z, α, β, γ]`, full float round-trip precision via `%.9g`).

**Inverse kinematics** — `ik_<MODEL>.json` for UR3/UR5/UR10 (end-effector dimension
0.0). 200 reachable-by-construction targets (FK of the first 200 FK inputs) plus 10
unreachable targets (position ≈ 10 m). Each case records the full **8×6** solution
matrix. `NaN` entries are stored as JSON **`null`** (valid JSON has no NaN literal) —
this encodes quirk **Q1** (unreachable targets silently produce NaN, and some solution
*families* are infeasible even for reachable poses).

## What is *not* captured, and why

- **Per-joint poses.** The v1.0 public API exposes only the tip pose
  (`forwardKinematics` returns it); the six intermediate joint poses live in private
  state with no accessor, and `operator<<` prints them only at default precision in
  degrees. Capturing them would require editing the library, which task 00 forbids
  (zero diffs under `cpp/`). We therefore characterize the **tip pose only**. A later
  task that adds accessors can extend the golden data.
- **`generateRandomReachablePose()` determinism.** It seeds from `std::random_device`
  and is non-deterministic by construction; property tests belong in task 06.

## Tolerances

Everything is 32-bit `float` (quirk Q9), so parity is checked with absolute tolerances,
also written into each JSON header:

| Check | Tolerance |
|---|---|
| FK tip position & Euler angles | `1e-5` (m / rad) |
| IK each solution joint | `1e-5` rad |
| IK NaN pattern | exact (`null` ⇔ `std::isnan`) |
| Round-trip: FK(IK solution).position vs target | `1e-4` m |

Round-trip compares **position only** — the FK-extract / IK-compose Euler conventions
are deliberately asymmetric (quirk Q5) and only round-trip through position.

## How it was generated

```
cmake -B build
cmake --build build --config Release
./build/tests/Release/golden_generator[.exe]   # writes tests/golden/*.json
```

The generator writes to the compiled-in `tests/golden` dir by default, or to a path
given as `argv[1]`.

## Regeneration policy

The `golden/*.json` files are **frozen artifacts of v1.0 behavior**. Regenerating them
would erase the safety net. **Do not regenerate** unless a roadmap task *explicitly*
mandates a deliberate behavior change (per the roadmap, only tasks **04e** input
validation and **04f** wrist singularity do). Such a task must call out the
regeneration, and the diff to `golden/*.json` is the reviewable record of the behavior
change.

## Running

```
ctest --test-dir build -C Release --output-on-failure
```
