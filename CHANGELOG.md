# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [2.0.0] — 2026-07-12

### Added
- `.github/workflows/ci.yml` — cross-platform GitHub Actions CI: builds and runs the golden characterization suite on {ubuntu, windows, macos} × {Debug, Release} (6 jobs, `fail-fast: false`), smoke-runs `ur_app`, and caches FetchContent downloads. CI status badge added to the README.
- `docs/REFERENCES.md` — full citations (with DOIs/links) for the literature previously vendored as PDFs under `Articles/`, plus the project dissertations, Jazar's textbook, and UR manufacturer docs.
- `tests/` — golden characterization suite (GoogleTest via FetchContent, CTest-discovered): FK/IK/round-trip parity tests replaying frozen v1.0 reference data in `tests/golden/*.json`, a one-shot `golden_generator`, and a dependency-free JSON reader. Zero changes to library code under `cpp/`.
- `CMakePresets.json` — `debug` and `release` presets (default generators, no toolchain files) making CMake the single, cross-platform build entry point.
- `.clang-format` — Microsoft-based style (Allman braces, tab indent, 120 col) applied once to `include/ src/ apps/ tests/` in a dedicated commit (`.git-blame-ignore-revs` entry included); enforced in CI via `clang-format --dry-run -Werror`.
- cppcheck (`--enable=warning,performance,portability --error-exitcode=1 --inline-suppr`) added to the static-analysis CI job.
- Warnings-as-errors (`-Wall -Wextra -Wpedantic` on GCC/Clang, `/W4` on MSVC) for `ur_kinematics`, `ur_demo`, and the test executables only, via `target_compile_options` on a `project_warnings` interface target — never applied globally, so FetchContent'd Eigen/GoogleTest are unaffected.
- Forward-looking unit and property-based tests (`tests/test_math_utils.cpp`, `test_robot_parameters.cpp`, `test_pose_and_fk_anchors.cpp`, `test_properties.cpp`): `rad`/`deg` and `calcTransformationMatrix` against hand-computed Modified-DH transforms; `RobotParameters` constants pinned against BASELINE.md's table; `pose`/`IkSolutions` helpers; FK all-zero-joint tip-position anchors (analytically derived, not computed by calling the library); a seeded FK∘IK round-trip property test; `generateRandomReachablePose()` properties; and an IK solution-family structural test (shoulder/wrist/elbow index semantics). All property tests use a fixed `std::mt19937` seed except `generateRandomReachablePose()` itself (non-deterministic by design).
- `UR_KINEMATICS_COVERAGE` CMake option: scopes `--coverage` (GCC/Clang) to `ur_kinematics` + its test suite only, via a `project_coverage` interface target (same pattern as `project_warnings`). The `coverage` CI job now configures with `-DUR_KINEMATICS_COVERAGE=ON` instead of blanket `CMAKE_CXX_FLAGS`/`CMAKE_EXE_LINKER_FLAGS`, so FetchContent'd Eigen/GoogleTest are no longer instrumented.

### Changed
- Project layout reorganized to a conventional `include/` / `src/` / `apps/` structure (behavior-preserving; golden results byte-identical): public headers live in `include/ur_kinematics/` (`ur_kinematics.h`, `robot_parameters.h`) and are consumed as `#include <ur_kinematics/ur_kinematics.h>`; the library implementation and the private `math_utils` helper (ex-`mathLib`, folded into the `universalRobots` namespace) live in `src/`; the demo app moved to `apps/demo/`. The build is now a single `ur_kinematics` static library plus the `ur_demo` executable (renamed from `ur_app`); the separate `math_lib` target and the nested `cpp/` tree are gone. The `ITERATIONS` benchmark macro became a `constexpr`. The `universalRobots` namespace is unchanged (any rename is a one-line follow-up).
- Internals/header hygiene (behavior-preserving, golden results unchanged): `m_endEffector` and `m_MDHmatrix` are now private; the 9×4 Modified-DH matrix is built in one place (`setMDHmatrix()`, called from the constructor) instead of three; scalar/enum parameters are passed by value instead of by `const` reference; the library header includes `<iosfwd>` instead of `<iostream>`; useless top-level `const` on by-value return types is dropped; `pose`/`joint` rely on default member initializers; and the frame/solution index masks in forward/inverse kinematics use named `constexpr` index sets. Dead commented code and a misdocumented axis comment were removed.
- **Breaking API change** — the public kinematics API is now strongly typed. `UR::forwardKinematics()` takes a `UR::JointVector` (`std::array<float, 6>`) instead of a reference-to-C-array; `UR::inverseKinematics()` returns the eight solutions by value as `UR::IkSolutions` (a struct wrapping `std::array<std::array<float, 6>, 8>`) instead of writing through a `float(*)[8][6]` out-parameter; `checkPoseReachability()` is renamed `isSolutionValid()` and takes a `std::array<float, 6>`. The value-returning functions are `[[nodiscard]]`. Semantics, solution order, and NaN propagation for unreachable poses are unchanged; golden results stay bit-identical.
- DH link dimensions moved from `#define UR*_LINK_DIMENSIONS_*` macros into a typed, single source of truth: `robotParameters.h` (`struct RobotParameters` + `constexpr kUR3/kUR5/kUR10` + `parametersFor()`). The `UR` constructor now copies `m_d`/`m_a` (now `std::array`) from `parametersFor(robotType)` instead of `memcpy` from macros. Values are bit-identical (same `float` literals); golden results unchanged.
- `main.cpp` — removed the trailing `std::cin.get()` so `ur_app` runs to completion without keyboard input (required for CI smoke-running the demo). Console FK/IK output is otherwise unchanged.
- Repository slimmed to the C++ library only. The MATLAB reference implementation, CoppeliaSim scenes/models, and `LAUNCH.md` moved to the archive repo [`Jgocunha/universal-robots-kinematics-matlab`](https://github.com/Jgocunha/universal-robots-kinematics-matlab); README updated to state the new scope and link the archive.
- Moved `Resources/errorSolsFlow.png` → `docs/images/errorSolsFlow.png`.
- `.gitignore` replaced with a proper C++/CMake/IDE ignore set (`build*/`, `out/`, `.vs/`, `.vscode/`, `*.user`, `CMakeUserPresets.json`, OS junk); `.claude/` kept ignored.
- `.clang-tidy` reconciled from a blanket 16-category disable to a curated `bugprone-*`/`modernize-*`/`readability-*`/`performance-*`/`cppcoreguidelines-*` set: every finding is fixed in-code, `NOLINT`'d with a justification, or its check is disabled with a documented reason. The fragile IK/FK solver math (task 04f) is protected from behavior-changing tidy fixes.

### Fixed
- `IkSolutions::valid` left uninitialized when default-constructed without `= {}` (cppcheck `uninitMemberVarNoCtor`).
- Benchmark demo: `std::chrono::duration` accumulators (`sumInvKin_us`, `sumFwdKin_us`, ...) were left uninitialized before their first `+=`, and an outer `tipPoseInput` was fully shadowed (and thus dead) by an inner loop-scoped variable of the same name — both caught by `-Wall`/`/W4` and cppcheck while wiring up warnings-as-errors.

### Removed
- CoppeliaSim remote-API integration: `Dependencies/CoppeliaSim/`, `Application/src/coppeliasimTests.{h,cpp}`, the vendored `extApi*`/`extApiPlatform*` sources, the `WITH_COPPELIASIM` CMake option, and the `#ifdef WITH_COPPELIASIM` blocks in `main.cpp`. The simulator integration lives in the archive repo [`Jgocunha/universal-robots-kinematics-matlab`](https://github.com/Jgocunha/universal-robots-kinematics-matlab).
- Visual Studio project artifacts (`*.vcxproj.filters`); CMake (via `CMakePresets.json`) is now the only build system. Removes the last of the hardcoded `C:\Work\Eigen` include paths that lived in the VS project files.
- `MATLAB files/`, `CoppeliaSim files/`, `LAUNCH.md` (relocated to the archive repo) and `Articles/` (copyrighted PDFs; replaced by `docs/REFERENCES.md`).
- `.github/ISSUE_TEMPLATE/bug_report.md` — bug report template
- `.github/ISSUE_TEMPLATE/feature_request.md` — feature request template
- `.github/PULL_REQUEST_TEMPLATE.md` — pull request template
- `.claude/feature-ci-cd-workflow.md` — feature: cross-platform CI/CD with GitHub Actions
- `.claude/feature-codecov-integration.md` — feature: code coverage via Codecov
- `.claude/feature-google-test.md` — feature: unit tests with Google Test
- `.claude/feature-release-workflow.md` — feature: automated release workflow
- `.claude/feature-static-analysis.md` — feature: automated static analysis (clang-tidy, cppcheck)
- `.claude/feature-doxygen-documentation.md` — feature: Doxygen API documentation
- `.claude/feature-wiki-documentation.md` — feature: GitHub Wiki
- `.claude/bug-hardcoded-eigen-path.md` — bug: hard-coded Eigen path in Visual Studio projects
- `.claude/bug-windows-only-build.md` — bug: no cross-platform build system
- `.claude/bug-no-input-validation.md` — bug: no input validation in kinematics API
- `.claude/bug-ik-degenerate-theta5.md` — bug: wrist singularity not handled correctly
- `.claude/bug-out-of-workspace-no-error.md` — bug: silent NaN on out-of-workspace poses
- `.claude/bug-readme-incomplete.md` — bug: C++ section of README is a placeholder

---

## [1.0.0] — 2021-10-24

First stable C++ release. Forward and inverse kinematics solver for UR3, UR5, and UR10 using the Modified Denavit-Hartenberg convention.

### Added
- `universalRobotsKinematics` library: `UR` class with `forwardKinematics()`, `inverseKinematics()`, `generateRandomReachablePose()`, and `checkPoseReachability()`
- `mathLib` library: `rad()`, `deg()`, and `calcTransformationMatrix()` utilities using Eigen
- `Application` project with `main.cpp` entry point
- CoppeliaSim C++ remote API integration (`coppeliasimTests.cpp`)
- Benchmarking harness (`benchmarking.cpp`) running 100,000 random poses; results documented in `README.md`
- Visual Studio 2019 solution (`universalRobotsKinematics.sln`)
- Support for UR3, UR5, and UR10 link dimension parameters

### Fixed
- Bug corrections to inverse kinematics and benchmarking
- Corrected Euler angles notation (ZYX convention)
- Fixed `atan2` error in joint angle calculation
- Theta3 sign correction for CoppeliaSim angle convention

---

## [0.3.0] — 2020-11-07

### Added
- `LAUNCH.md` with step-by-step CoppeliaSim simulation launch instructions
- CoppeliaSim simulation README for the MATLAB integration

### Fixed
- Bug fixes from `bug-fix` branch: corrected link dimensions and joint angle computations

---

## [0.2.0] — 2020-10-18

Stable MATLAB + CoppeliaSim version with end-effector position and UR10 link dimensions.

### Added
- Stable v3 MATLAB solution
- `RPY.m` rotation matrix utility with improved Euler angle handling
- Validation script computing solution accuracy over 100,000 poses
- Benchmarking: computation times exported to Excel
- MATLAB GUI with text-box inputs for joint angles and robot model selection
- CoppeliaSim scenes for UR3, UR5, and UR10

### Changed
- Link dimensions updated to match actual UR10 e-series hardware measurements
- Output files redirected to a dedicated output directory

### Fixed
- `atan2` singularity in joint 2 calculation
- Corrected end-effector position in forward kinematics

---

## [0.1.0] — 2020-10-15

Initial commit. MATLAB forward and inverse kinematics for UR10 with CoppeliaSim integration.

### Added
- MATLAB kinematics scripts (`main.m`, `RPY.m`)
- CoppeliaSim scene files and Lua scripts
- Articles and literature folder
- Initial `README.md`
- `LICENSE`

[2.0.0]: https://github.com/Jgocunha/universal-robots-kinematics/compare/v1.0.0...v2.0.0
[1.0.0]: https://github.com/Jgocunha/universal-robots-kinematics/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/Jgocunha/universal-robots-kinematics/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/Jgocunha/universal-robots-kinematics/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/Jgocunha/universal-robots-kinematics/releases/tag/v0.1.0
