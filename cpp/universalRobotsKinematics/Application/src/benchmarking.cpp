// benchmarking.cpp

#include "benchmarking.h"


namespace benchmark
{
	void runBenchmarkTests(universalRobots::UR& robot)
	{
		std::cout << "Benchmarking..." << std::endl << std::endl;

		float ikSols[robot.m_numIkSol][robot.m_numDoF] = {};
		universalRobots::pose tipPoseInput = {};
		universalRobots::pose tipPoseOutput = {};
		universalRobots::pose poseError = {};
		universalRobots::pose avgPoseError = {};

		std::chrono::duration<double, std::milli> invKin_ms;
		std::chrono::duration<double, std::milli> sumInvKin_ms;
		std::chrono::duration<double, std::milli> avgInvKin_ms;

		std::chrono::duration<double, std::milli> fwdKin_ms;
		std::chrono::duration<double, std::milli> sumFwdKin_ms;
		std::chrono::duration<double, std::milli> avgFwdKin_ms;

		for (unsigned int i = 0; i < ITERATIONS; i++)
		{
			// generate a random pose
			universalRobots::pose tipPoseInput = robot.generateRandomReachablePose();
			std::chrono::time_point startInvKin = std::chrono::high_resolution_clock::now();
			robot.inverseKinematics(tipPoseInput, &ikSols);
			std::chrono::time_point endInvKin = std::chrono::high_resolution_clock::now();
			invKin_ms = (endInvKin - startInvKin); // Getting number of milliseconds as a double.
			sumInvKin_ms = sumInvKin_ms + invKin_ms;

			//std::cout << "Inverse kinematics" << std::endl;
			//for (unsigned int i = 0; i < robot.m_numIkSol; i++)
			//	std::cout << "IK solution " << i << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
			//	mathLib::deg(ikSols[i][3]) << " " << mathLib::deg(ikSols[i][4]) << " " << mathLib::deg(ikSols[i][5]) << std::endl;

			for (unsigned int j = 0; j < robot.m_numIkSol; j++)
			{
				std::chrono::time_point startFwdKin = std::chrono::high_resolution_clock::now();
				tipPoseOutput = robot.forwardKinematics(ikSols[j]);
				std::chrono::time_point endFwdKin = std::chrono::high_resolution_clock::now();
				fwdKin_ms = (endFwdKin - startFwdKin); // Getting number of milliseconds as a double.
				sumFwdKin_ms = sumFwdKin_ms + fwdKin_ms;
				poseError = tipPoseInput - tipPoseOutput;
			}
		}
		avgInvKin_ms = sumInvKin_ms / ITERATIONS;
		avgFwdKin_ms = sumFwdKin_ms / (ITERATIONS * robot.m_numIkSol);
		avgPoseError = poseError / (ITERATIONS * robot.m_numIkSol);

		std::cout << "Generated " << ITERATIONS << " random valid tip poses for " << robot.getRobotType() << "..." << std::endl;
		std::cout << "Generated eight inverse kinematic solutions for each tip pose..." << std::endl;
		std::cout << "Average IK function execution time: " << avgInvKin_ms.count() << "ms" << std::endl;
		std::cout << "Each IK sol (" << ITERATIONS * robot.m_numIkSol << ") has been run through forward kinematics..." << std::endl;
		std::cout << "Average FK function execution time: " << avgFwdKin_ms.count() << "ms" << std::endl;
		std::cout << "error = [tip_pose_input - tip_pose_output]" << std::endl;
		std::cout << "average error = " << avgPoseError.m_pos[0] << " " << avgPoseError.m_pos[1] << " " << avgPoseError.m_pos[2] << " "
			<< avgPoseError.m_eulerAngles[0] << " " << avgPoseError.m_eulerAngles[1] << " " << avgPoseError.m_eulerAngles[2] << std::endl;
	}

} // namespace benchmark