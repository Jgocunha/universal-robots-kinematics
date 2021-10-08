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

	std::cout << robot;

	std::cin.get();
}