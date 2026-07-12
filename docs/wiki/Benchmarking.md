# Benchmarking

Benchmarking consists of two parts:

1. measuring the [compute time](https://stackoverflow.com/questions/22387586/measuring-execution-time-of-a-function-in-c) of the forward and inverse kinematics functions;
2. measuring the round-trip error of the solutions (FK ∘ IK).

The published results were produced over a set of 100,000 randomly generated target tip poses (generated via `UR::generateRandomReachablePose()`), run on a Ryzen 5 3600 CPU at 4.28 GHz. The harness lives in `apps/demo/benchmarking.cpp` and is run from `ur_demo`.

## Average computation times

|  | MATLAB | C++ |
|---|---|---|
| Forward Kinematics | 1.832571E-05 s | 1.39434E-06 s |
| Inverse Kinematics | 1.612797E-04 s | 1.07403E-05 s |

MATLAB's compute times are roughly 10× slower than the C++ solution.

## Average error

Error is computed by round-tripping a random reachable pose through inverse kinematics and back through forward kinematics, then comparing against the original:

![How to get the errors](https://raw.githubusercontent.com/Jgocunha/universal-robots-kinematics/main/docs/images/error-calculation.png)

|  | *x* | *y* | *z* | *alpha* | *beta* | *gamma* |
|---|---|---|---|---|---|---|
| average error | 7.45E-14 | 1.86E-14 | 7.45E-14 | 4.14E-06 | 4.14E-06 | 4.14E-06 |

**Units**: *x, y, z* in metres; *alpha, beta, gamma* in radians.

## Reproducing the results

Build and run the demo (see [Getting Started](Getting-Started)):

```bash
cmake --preset release
cmake --build --preset release
./build/release/ur_demo          # Linux/macOS
build\release\Release\ur_demo.exe # Windows
```

The demo runs the benchmarking harness and prints timing and error statistics to the console.
