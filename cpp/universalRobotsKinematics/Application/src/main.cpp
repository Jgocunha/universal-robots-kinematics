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

	std::cout << robot << std::endl << std::endl;
	
	// test different target tip poses
	// 
	//float targetTipPose[6] = { 0.2982f, 0.6746f, 0.7211f, mathLib::rad(180), mathLib::rad(45), mathLib::rad(90) };
	//const Eigen::Matrix<float, 1, 6> targetTipPose = { 0.2982f, 0.6746f, 0.7211f, mathLib::rad(180), mathLib::rad(45), mathLib::rad(90) };
	const Eigen::Matrix<float, 1, 6> targetTipPose = { 0.3835f, 0.3730f, 0.9581f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) }; // mathLib::rad(23), mathLib::rad(345), mathLib::rad(78), mathLib::rad(66), mathLib::rad(77), mathLib::rad(12)
	// compute inverse kinematics 
	//float ikSols[8][6] = 
		robot.inverseKinematics(targetTipPose);
	
	//for (unsigned int i = 0; i < robot.m_numIkSol; i++)
	//	std::cout << "IK solution " << i << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
	//	mathLib::deg(ikSols[i][3]) << " " << mathLib::deg(ikSols[i][4]) << " " << mathLib::deg(ikSols[i][5]) << std::endl;

	
	//std::cout << robot.inverseKinematics(targetTipPose)[0][0];

	std::cin.get();
}