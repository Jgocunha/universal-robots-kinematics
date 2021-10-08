#include <iostream>
#include "universalRobotsKinematics.h"

// TO DO:
// 1. getMDHmatrix()
// 2. setMDHmatrix() without temp. matrix
// 3. calcTransformationMatrix without temp. matrix


int main()
{
	universalRobots::UR robot;

	//float targetJointValues[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	//float targetJointValues[6] = { mathLib::rad(90), mathLib::rad(45), mathLib::rad(90), mathLib::rad(45), mathLib::rad(45), mathLib::rad(90) };

	float targetJointValues[6] = { mathLib::rad(23), mathLib::rad(345), mathLib::rad(78), mathLib::rad(66), mathLib::rad(77), mathLib::rad(12) };

	// give joint states, receive end-effector pose
	robot.setTipPose(robot.forwardKinematics(targetJointValues));

	std::cout << robot;

	std::cin.get();
}