// benchmarking.h

#pragma once

#define ITERATIONS 5000

#include <chrono>
#include "universalRobotsKinematics.h"

// Benchmarking consists of:
// 1. getting compute times of forward and inverse kinematics functions
// 2. getting the error of the solutions

// https://stackoverflow.com/questions/22387586/measuring-execution-time-of-a-function-in-c

namespace benchmark
{
	void runBenchmarkTests(universalRobots::UR& robot);

} // namespace benchmark
