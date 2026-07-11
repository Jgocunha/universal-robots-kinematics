// main.cpp

#include <iostream>
#include <ur_kinematics/ur_kinematics.h>
#include "benchmarking.h"

// The whole body is wrapped in a try/catch(const std::exception&)/catch(...),
// which is exhaustive; the checker can't see that catch(...) covers everything.
// NOLINTNEXTLINE(bugprone-exception-escape)
int main()
{
	try
	{
		// instatiate a UR object
		//
		universalRobots::UR robot(universalRobots::URtype::UR5);

		// test different target joint values
		//
		const universalRobots::UR::JointVector targetJointValues = {universalRobots::rad(23), universalRobots::rad(345),
																	universalRobots::rad(78), universalRobots::rad(66),
																	universalRobots::rad(77), universalRobots::rad(12)};

		// compute forward kinematics and update robot's tip pose
		//
		const universalRobots::pose targetTipPose = robot.forwardKinematics(targetJointValues);

		std::cout << robot << '\n';

		// compute inverse kinematics
		//
		const universalRobots::UR::IkSolutions ikSols = robot.inverseKinematics(targetTipPose);

		std::cout << "Inverse kinematics" << '\n';
		for (unsigned int i = 0; i < universalRobots::UR::m_numIkSol; i++)
			std::cout << "IK solution " << i + 1 << ": " << universalRobots::deg(ikSols.solutions[i][0]) << " "
					  << universalRobots::deg(ikSols.solutions[i][1]) << " "
					  << universalRobots::deg(ikSols.solutions[i][2]) << " "
					  << universalRobots::deg(ikSols.solutions[i][3]) << " "
					  << universalRobots::deg(ikSols.solutions[i][4]) << " "
					  << universalRobots::deg(ikSols.solutions[i][5]) << '\n';

		// benchmarking
		//
		universalRobots::UR benchmarkRobot(universalRobots::URtype::UR5);
		benchmark::runBenchmarkTests(benchmarkRobot);
	}
	catch (const std::exception& e)
	{
		std::cerr << "ur_demo failed: " << e.what() << '\n';
		return 1;
	}
	catch (...)
	{
		std::cerr << "ur_demo failed: unknown exception" << '\n';
		return 1;
	}

	return 0;
}