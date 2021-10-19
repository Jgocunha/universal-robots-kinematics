// benchmarking.h

#pragma once

#define ITERATIONS 100000

#include <chrono>
#include "universalRobotsKinematics.h"

// Benchmarking consists of:
// 1. getting compute times of forward and inverse kinematics functions
// 2. getting the error of the solutions

// https://stackoverflow.com/questions/22387586/measuring-execution-time-of-a-function-in-c

namespace benchmark
{
	class timer
	{
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
		std::chrono::time_point<std::chrono::high_resolution_clock> m_endTimePoint;
	public:
		timer()
		{
			m_startTimePoint = std::chrono::high_resolution_clock::now();
		}
		std::chrono::microseconds stop();
	};

	void runBenchmarkTests(universalRobots::UR& robot);

} // namespace benchmark
