// coppeliasimTests.cpp

<<<<<<< HEAD
=======
#ifndef _DEBUG

>>>>>>> cpp-test
#include <iostream>
#include "coppeliasimTests.h"

namespace coppeliaSim
{
	void retrieveRobotJointHandles(const int& clientID, const std::string& jointHandleName, const unsigned int& numJoints, int (*out_robotJointHandles)[])
	{
		for (unsigned int i = 0; i < numJoints; i++)
			simxGetObjectHandle(clientID, (jointHandleName + std::to_string(i + 1)).c_str(), &(*out_robotJointHandles)[i], simx_opmode_oneshot_wait);
	}

	void sendRobotTargetJointValues(const int& clientID, const universalRobots::UR& robot, const int (&robotJointHandles)[], const float (&jointValue)[], const unsigned int& waitTime)
	{
			for (unsigned int j = 0; j < robot.m_numDoF; j++)
				simxSetJointTargetPosition(clientID, robotJointHandles[j], jointValue[j], simx_opmode_blocking);
			Sleep(waitTime);
	}

	std::string getJointHandleName(const universalRobots::UR& robot)
	{
		switch (robot.getRobotType())
		{
		case universalRobots::URtype::UR3:
			return "UR3_joint";
		case universalRobots::URtype::UR5:
			return "UR5_joint";
		case universalRobots::URtype::UR10:
			return "UR10_joint";
		default:
			return "UR10_joint";
		}
	}

<<<<<<< HEAD
	void runCoppeliaSimTests(universalRobots::UR& robot, const float (&targetTipPose)[])
=======
	void runCoppeliaSimTests(universalRobots::UR& robot, const universalRobots::pose& targetTipPose)
>>>>>>> cpp-test
	{
		simxFinish(-1); // Just in case, close all opened connections
		const int clientID = simxStart((simxChar*)"127.0.0.1", 19999, true, true, 5000, 5); // start connection

		if (clientID > -1)
		{
			std::cout << "\nConnected to CoppeliaSim scene!" << std::endl << std::endl;

			const std::string jointHandleName = getJointHandleName(robot);

			// retrieve joint handles from CoppeliaSim
			int robotJointHandles[robot.m_numDoF] = {};
			retrieveRobotJointHandles(clientID, jointHandleName, robot.m_numDoF, &robotJointHandles);

			// compute inverse kinematics
			float ikSols[robot.m_numIkSol][robot.m_numDoF] = {};
			robot.inverseKinematics(targetTipPose, &ikSols);

			// send joint values to CoppeliaSim
			for (unsigned int i = 0; i < robot.m_numIkSol; i++)
			{
<<<<<<< HEAD
				std::cout << "IK solution " << i << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
=======
				std::cout << "IK solution " << i + 1 << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
>>>>>>> cpp-test
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

<<<<<<< HEAD
} // namespace coppeliaSim
=======
} // namespace coppeliaSim

#endif
>>>>>>> cpp-test
