// main.cpp

#include <iostream>
#include "universalRobotsKinematics.h"
#include "benchmarking.h"


int main()
{
	// instatiate a UR object
	//
	universalRobots::UR robot(universalRobots::URtype::UR5);

	// test different target joint values
	//
	const universalRobots::UR::JointVector targetJointValues = { mathLib::rad(23), mathLib::rad(345), mathLib::rad(78), mathLib::rad(66), mathLib::rad(77), mathLib::rad(12) };

	// compute forward kinematics and update robot's tip pose
	//
	const universalRobots::pose targetTipPose = robot.forwardKinematics(targetJointValues);

	std::cout << robot << std::endl;

	// compute inverse kinematics
	//
	const universalRobots::UR::IkSolutions ikSols = robot.inverseKinematics(targetTipPose);

	std::cout << "Inverse kinematics" << std::endl;
	for (unsigned int i = 0; i < robot.m_numIkSol; i++)
		std::cout << "IK solution " << i + 1 << ": " << mathLib::deg(ikSols.solutions[i][0]) << " " << mathLib::deg(ikSols.solutions[i][1]) << " " << mathLib::deg(ikSols.solutions[i][2]) << " " <<
		mathLib::deg(ikSols.solutions[i][3]) << " " << mathLib::deg(ikSols.solutions[i][4]) << " " << mathLib::deg(ikSols.solutions[i][5]) << std::endl;

	// benchmarking
	//
	universalRobots::UR benchmarkRobot(universalRobots::URtype::UR5);
	benchmark::runBenchmarkTests(benchmarkRobot);

	return 0;
}