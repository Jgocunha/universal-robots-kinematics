// coppeliasimTests.h

#pragma once

#include <windows.h>
#include "universalRobotsKinematics.h"

extern "C"
{
	#include "extApi.h"
}

#ifdef _WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

namespace coppeliaSim
{	
	void runCoppeliaSimTests(universalRobots::UR& robot);
	void retrieveRobotJointHandles(const int& clientID, const std::string& jointHandleName, const unsigned int& numJoints, int(*out_robotJointHandles)[]);
	void sendRobotTargetJointValues(const int& clientID, const universalRobots::UR& robot, const int(&robotJointHandles)[], const float(&jointValue)[], const unsigned int& waitTime = 500);
}