# Getting Started

## Requirements

- CMake 3.21+ (for presets; the build itself needs 3.20+)
- A C++20 compiler: GCC 10+, Clang 12+, or MSVC 19.29+
- Git — [Eigen](#dependencies) and [GoogleTest](#dependencies) are fetched from GitHub at configure time, so no manual installation is needed for either.

Supported platforms: Windows, Linux, macOS.

## Dependencies

Both dependencies are fetched automatically by CMake's `FetchContent` at configure time:

- **[Eigen 3.4.0](https://eigen.tuxfamily.org/)** — linear algebra (matrices/vectors) used throughout the kinematics solver.
- **[GoogleTest v1.15.2](https://github.com/google/googletest)** — used to build the test suite only.

## Building

The build uses CMake presets (`debug`, `release`) with the default generator on each platform.

```bash
# Clone
git clone https://github.com/Jgocunha/universal-robots-kinematics.git
cd universal-robots-kinematics

# Configure and build (Eigen and GoogleTest are downloaded automatically)
cmake --preset release
cmake --build --preset release

# Run the demo
./build/release/ur_demo          # Linux/macOS
build\release\Release\ur_demo.exe # Windows
```

Swap `release` for `debug` to build the debug configuration under `build/debug`.

## Running the tests

```bash
ctest --test-dir build/release --output-on-failure   # add -C Release on Windows/multi-config
```

### Coverage (GCC/Clang only)

```bash
cmake -B build/coverage -DCMAKE_BUILD_TYPE=Debug -DUR_KINEMATICS_COVERAGE=ON
cmake --build build/coverage
ctest --test-dir build/coverage --output-on-failure
gcovr --root . --filter 'include/.*' --filter 'src/.*' --object-directory build/coverage
```

## Using the library in your own project

Instantiate a robot and call `forwardKinematics` or `inverseKinematics`:

```cpp
#include <ur_kinematics/ur_kinematics.h>

universalRobots::UR robot(universalRobots::URtype::UR5);

// Forward kinematics
const universalRobots::UR::JointVector joints = { 0.4f, -1.0f, 1.2f, 0.3f, 0.8f, 0.2f }; // radians
universalRobots::pose tip = robot.forwardKinematics(joints);

// Inverse kinematics (up to 8 solutions)
const universalRobots::UR::IkSolutions ikSols = robot.inverseKinematics(tip);

// ikSols.valid[i] tells you whether ikSols.solutions[i] is a real solution
const bool hasSolution = ikSols.anyValid();
```

See the [UR Class API](UR-Class-API) page for the full public interface.
