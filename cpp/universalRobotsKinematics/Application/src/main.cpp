// main.cpp

#include <iostream>
#include "universalRobotsKinematics.h"


int main()
{
	// instatiate a UR object
	universalRobots::UR robot;

	// test different target joint values
	// 
	//float targetJointValues[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	//float targetJointValues[6] = { mathLib::rad(90), mathLib::rad(45), mathLib::rad(90), mathLib::rad(45), mathLib::rad(45), mathLib::rad(90) };
	float targetJointValues[6] = { mathLib::rad(23), mathLib::rad(345), mathLib::rad(78), mathLib::rad(66), mathLib::rad(77), mathLib::rad(12) };

	// compute forward kinematics and update robot's tip pose
	robot.setTipPose(robot.forwardKinematics(targetJointValues));


	// test different target tip poses
	// 
	//float targetTipPose[6] = { 0.2982f, 0.6746f, 0.7211f, mathLib::rad(180), mathLib::rad(45), mathLib::rad(90) };
	const Eigen::Matrix<float, 1, 6> targetTipPose = { 0.2982f, 0.6746f, 0.7211f, mathLib::rad(180), mathLib::rad(45), mathLib::rad(90) };
	// compute inverse kinematics 
	//float ikSols[8][robot.m_numDoF] =
		

	std::cout << robot << std::endl << std::endl;
	//std::cout << robot.inverseKinematics(targetTipPose)[0][0];

	std::cin.get();
}