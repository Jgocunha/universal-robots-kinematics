// coppeliasimTests.cpp

#include <iostream>
#include "coppeliasimTests.h"

// https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm
// This is not enough!!! You need to include additional directories { remoteApi and include folder }
// and also https://stackoverflow.com/questions/14386/fopen-deprecated-warning
// It looks like Microsoft has deprecated lots of calls which use buffers to improve code security.However, the solutions they're providing aren't portable.Anyway, if you aren't interested in using the secure version of their calls (like fopen_s), you need to place a definition of _CRT_SECURE_NO_DEPRECATE before your included header files. For example:

// Run the main application in release mode.

namespace coppeliaSim
{
	void retrieveRobotJointHandles(const int& clientID, const std::string& jointHandleName, const unsigned int& numJoints, int (*out_robotJointHandles)[])
	{
		for (unsigned int i = 0; i < numJoints; i++)
			simxGetObjectHandle(clientID, (jointHandleName + std::to_string(i + 1)).c_str(), &(*out_robotJointHandles)[i], simx_opmode_oneshot_wait);

	}

	void sendRobotTargetJointValues(const int& clientID, const universalRobots::UR& robot, const int (&robotJointHandles)[], const float (&jointValue)[], const unsigned int& waitTime = 500)
	{
			for (unsigned int j = 0; j < robot.m_numDoF; j++)
				simxSetJointTargetPosition(clientID, robotJointHandles[j], jointValue[j], simx_opmode_blocking);
			Sleep(waitTime);
	}

	void runCoppeliaSimTests(universalRobots::UR& robot)
	{
		simxFinish(-1); // Just in case, close all opened connections
		const int clientID = simxStart((simxChar*)"127.0.0.1", 19999, true, true, 5000, 5); // start connection

		if (clientID > -1)
		{
			std::cout << "\nConnected to CoppeliaSim scene!" << std::endl;

			const std::string jointHandleName = "UR5_joint";

			// retrieve joint handles from CoppeliaSim
			int robotJointHandles[robot.m_numDoF] = {};
			retrieveRobotJointHandles(clientID, jointHandleName, robot.m_numDoF, &robotJointHandles);


			// test different target tip poses
			// 
			//const float targetTipPose[robot.m_numDoF] = { 0.3835f, 0.3730f, 0.9581f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) }; // UR10
			const float targetTipPose[robot.m_numDoF] = { 0.2847f, 0.2606f, 0.6663f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) }; // UR5
			//const float targetTipPose[robot.m_numDoF] = { 0.1730f, 0.2153f, 0.4492f, mathLib::rad(-30.5348f), mathLib::rad(28.4603f), mathLib::rad(47.5405f) }; // UR3

			// compute inverse kinematics
			//
			float ikSols[robot.m_numIkSol][robot.m_numDoF] = {};
			robot.inverseKinematics(targetTipPose, &ikSols);


			// send joint values to CoppeliaSim
			for (unsigned int i = 0; i < robot.m_numIkSol; i++)
			{
				std::cout << "IK solution " << i << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
					mathLib::deg(ikSols[i][3]) << " " << mathLib::deg(ikSols[i][4]) << " " << mathLib::deg(ikSols[i][5]) << std::endl;
				sendRobotTargetJointValues(clientID, robot, robotJointHandles, ikSols[i], 500);
			}

			// allow Coppelia to display tip pose
			simxSetIntegerSignal(clientID, "showPos", 1, simx_opmode_blocking);
			Sleep(1);
			simxSetIntegerSignal(clientID, "showPos", 0, simx_opmode_blocking);
		}
		else
			std::cout << "Failed to connect to CoppeliaSim scene!" << std::endl;



		simxFinish(clientID);

	}




}