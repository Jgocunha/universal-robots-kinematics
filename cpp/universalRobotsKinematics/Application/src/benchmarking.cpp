// benchmarking.cpp

#include "benchmarking.h"


namespace benchmark
{
	void runBenchmarkTests(universalRobots::UR& robot)
	{
		std::cout << "____________________________\nBenchmarking..." << std::endl << std::endl;

		float ikSols[robot.m_numIkSol][robot.m_numDoF] = {};
		universalRobots::pose tipPoseInput = {};
		universalRobots::pose tipPoseOutput = {};
		universalRobots::pose poseError = {};
		universalRobots::pose avgPoseError = {};

		std::chrono::duration<double, std::micro> invKin_us;
		std::chrono::duration<double, std::micro> sumInvKin_us;
		std::chrono::duration<double, std::micro> avgInvKin_us;

		std::chrono::duration<double, std::micro> fwdKin_us;
		std::chrono::duration<double, std::micro> sumFwdKin_us;
		std::chrono::duration<double, std::micro> avgFwdKin_us;

		for (unsigned int i = 0; i < ITERATIONS; i++)
		{
			universalRobots::pose tipPoseInput = robot.generateRandomReachablePose();
			// generate a random pose
			{
				timer ikTimer;
				robot.inverseKinematics(tipPoseInput, &ikSols);
				invKin_us = ikTimer.stop(); // Getting number of microseconds as a double.
			}
			sumInvKin_us = sumInvKin_us + invKin_us;

			for (unsigned int j = 0; j < robot.m_numIkSol; j++)
			{
				{
					timer fkTimer;
					tipPoseOutput = robot.forwardKinematics(ikSols[j]);
					fwdKin_us = fkTimer.stop(); // Getting number of microseconds as a double.
				}
				sumFwdKin_us = sumFwdKin_us + fwdKin_us;
				poseError = tipPoseInput - tipPoseOutput;
			}
		}
		avgInvKin_us = sumInvKin_us / ITERATIONS;
		avgFwdKin_us = sumFwdKin_us / (ITERATIONS * robot.m_numIkSol);
		avgPoseError = poseError / (ITERATIONS * robot.m_numIkSol);

		std::cout << "Generated " << ITERATIONS << " random valid tip poses for " << robot.getRobotType() << "..." << std::endl;
		std::cout << "Generated eight inverse kinematic solutions for each tip pose..." << std::endl;
		std::cout << "Average IK function execution time: " << avgInvKin_us.count() << "us" << std::endl;
		std::cout << "Each IK sol (" << ITERATIONS * robot.m_numIkSol << ") has been run through forward kinematics..." << std::endl;
		std::cout << "Average FK function execution time: " << avgFwdKin_us.count() << "us" << std::endl;
		std::cout << "error = [tip_pose_input - tip_pose_output]" << std::endl;
		std::cout << "average error = " << avgPoseError.m_pos[0] << " " << avgPoseError.m_pos[1] << " " << avgPoseError.m_pos[2] << " "
			<< avgPoseError.m_eulerAngles[0] << " " << avgPoseError.m_eulerAngles[1] << " " << avgPoseError.m_eulerAngles[2] << std::endl;
	}

	std::chrono::microseconds timer::stop()
	{
		m_endTimePoint = std::chrono::high_resolution_clock::now();

		auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTimePoint);
		auto end = std::chrono::time_point_cast<std::chrono::microseconds>(m_endTimePoint);

		return end - start;		
	}

} // namespace benchmark