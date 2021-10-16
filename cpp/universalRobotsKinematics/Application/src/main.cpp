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
	//const universalRobots::pose targetTipPose( 0.3835f, 0.3730f, 0.9581f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) ); // UR10
	const universalRobots::pose targetTipPose = robot.forwardKinematics(targetJointValues); // UR5
	//const universalRobots::pose targetTipPose( 0.1730f, 0.2153f, 0.4492f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) ); // UR3

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
	benchmark::runBenchmarkTests(robot);

	std::cin.get();
}