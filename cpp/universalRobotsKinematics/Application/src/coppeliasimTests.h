// coppeliasimTests.h

#pragma once

#include <windows.h>
#include "universalRobotsKinematics.h"

extern "C"
{
	#include "extApi.h"
}

// https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm
// This is not enough!!! You need to include additional directories { remoteApi and include folder }
// and also https://stackoverflow.com/questions/14386/fopen-deprecated-warning
// It looks like Microsoft has deprecated lots of calls which use buffers to improve code security.However, the solutions they're providing aren't portable. Anyway, 
// if you aren't interested in using the secure version of their calls (like fopen_s), you need to place a definition of _CRT_SECURE_NO_DEPRECATE before your included header files. 
// For example:

#ifdef _WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

// Run the main application in release mode.


namespace coppeliaSim
{	
	/// <summary>
	/// Runs the main test. 
	/// 1st starts the connection.
	/// 2nd retrieves the robot's joint handles.
	/// 3rd computes the eight inverse kinematics solutions.
	/// 4th actuates the target solutions in CoppeliaSim.
	/// </summary>
	/// <param name="robot"></param>
	void runCoppeliaSimTests(universalRobots::UR& robot, const float(&targetTipPose)[]);

	/// <summary>
	/// Returns URtype + _joint
	/// </summary>
	/// <param name="robot"></param>
	/// <returns>robotJointHandleName</returns>
	std::string getJointHandleName(const universalRobots::UR& robot);


	/// <summary>
	/// Retreives the joint handles in CoppeliaSim.
	/// </summary>
	/// <param name="clientID"></param>
	/// <param name="jointHandleName"></param>
	/// <param name="numJoints"></param>
	/// <param name="out_robotJointHandles"></param>
	void retrieveRobotJointHandles(const int& clientID, const std::string& jointHandleName, const unsigned int& numJoints, int(*out_robotJointHandles)[]);

	/// <summary>
	/// Sets the joint angles of the robot in CoppeliaSim.
	/// </summary>
	/// <param name="clientID"></param>
	/// <param name="robot"></param>
	/// <param name="robotJointHandles"></param>
	/// <param name="jointValue"></param>
	/// <param name="waitTime"></param>
	void sendRobotTargetJointValues(const int& clientID, const universalRobots::UR& robot, const int(&robotJointHandles)[], const float(&jointValue)[], const unsigned int& waitTime = 500);

} // namespace coppeliaSim 