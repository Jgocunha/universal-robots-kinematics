// main.cpp

#include <iostream>
#include "universalRobotsKinematics.h"
#include "coppeliaSimTests.h"
#include "benchmarking.h"


int main()
{
	// instatiate a UR object
	//
	universalRobots::UR robot(universalRobots::URtype::UR5);

	// test different target joint values
	//
	const float targetJointValues[robot.m_numDoF] = { mathLib::rad(23), mathLib::rad(345), mathLib::rad(78), mathLib::rad(66), mathLib::rad(77), mathLib::rad(12) };

	// compute forward kinematics and update robot's tip pose
	//
	robot.forwardKinematics(targetJointValues);
	
	std::cout << robot << std::endl;

	// test different target tip poses
	// 
	const universalRobots::pose targetTipPose = robot.forwardKinematics(targetJointValues);

	// compute inverse kinematics
	//
	float ikSols[robot.m_numIkSol][robot.m_numDoF] = {};
	robot.inverseKinematics(targetTipPose, &ikSols);

	std::cout << "Inverse kinematics" << std::endl;
	for (unsigned int i = 0; i < robot.m_numIkSol; i++)
		std::cout << "IK solution " << i + 1 << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
		mathLib::deg(ikSols[i][3]) << " " << mathLib::deg(ikSols[i][4]) << " " << mathLib::deg(ikSols[i][5]) << std::endl;

	// coppeliasim test
	//
	coppeliaSim::runCoppeliaSimTests(robot, targetTipPose);

	// benchmarking
	//
	universalRobots::UR benchmarkRobot(universalRobots::URtype::UR5);
	benchmark::runBenchmarkTests(benchmarkRobot);

	std::cin.get();

	return 0;
}