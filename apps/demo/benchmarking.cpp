// benchmarking.cpp

#include <algorithm>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "benchmarking.h"

namespace benchmark
{
	namespace
	{
		// Reconstructs the rotation matrix implied by a pose's stored Euler triple,
		// using the SAME convention forwardKinematics()'s FK-extract uses (the
		// pose(pos, Eigen::Matrix3f) ctor: eulerAngles(1,2,0) remapped to
		// {gamma,beta,alpha} order), which Eigen guarantees is an exact inverse of
		// that extraction. Both poses compared here (tipPoseInput and tipPoseOutput)
		// come from forwardKinematics(), so this single convention applies to both --
		// see #54.
		Eigen::Matrix3f reconstructRotation(const universalRobots::pose& p)
		{
			return Eigen::Matrix3f(Eigen::AngleAxisf(p.m_eulerAngles[2], Eigen::Vector3f::UnitY()) *
									Eigen::AngleAxisf(p.m_eulerAngles[1], Eigen::Vector3f::UnitZ()) *
									Eigen::AngleAxisf(p.m_eulerAngles[0], Eigen::Vector3f::UnitX()));
		}

		// Rotation-invariant SO(3) geodesic distance (radians) between two rotation
		// matrices. Unlike a raw Euler-component subtraction, this is not sensitive to
		// the two-fold Euler representation ambiguity, so it reports honest orientation
		// error (see #54).
		float geodesicOrientationError(const universalRobots::pose& a, const universalRobots::pose& b)
		{
			const Eigen::Matrix3f relative = reconstructRotation(a).transpose() * reconstructRotation(b);
			const float cosTheta = std::clamp((relative.trace() - 1.0f) / 2.0f, -1.0f, 1.0f);
			return std::acos(cosTheta);
		}
	} // namespace

	void runBenchmarkTests(universalRobots::UR& robot)
	{
		std::cout << "____________________________\nBenchmarking..." << '\n' << '\n';

		universalRobots::UR::IkSolutions ikSols = {};
		universalRobots::pose tipPoseOutput = {};
		universalRobots::pose poseError = {};
		double sumAbsPos[3] = {0.0, 0.0, 0.0};
		double sumOrientationError = 0.0;
		universalRobots::pose avgPoseError = {};

		std::chrono::duration<double, std::micro> invKin_us{};
		std::chrono::duration<double, std::micro> sumInvKin_us{};
		std::chrono::duration<double, std::micro> avgInvKin_us{};

		std::chrono::duration<double, std::micro> fwdKin_us{};
		std::chrono::duration<double, std::micro> sumFwdKin_us{};
		std::chrono::duration<double, std::micro> avgFwdKin_us{};

		unsigned int fkCalls = 0;

		for (unsigned int i = 0; i < ITERATIONS; i++)
		{
			universalRobots::pose tipPoseInput = robot.generateRandomReachablePose();
			// generate a random pose
			{
				timer ikTimer;
				ikSols = robot.inverseKinematics(tipPoseInput);
				invKin_us = ikTimer.stop(); // Getting number of microseconds as a double.
			}
			sumInvKin_us = sumInvKin_us + invKin_us;

			for (unsigned int j = 0; j < universalRobots::UR::m_numIkSol; j++)
			{
				if (!ikSols.valid[j])
					continue;

				{
					timer fkTimer;
					tipPoseOutput = robot.forwardKinematics(ikSols.solutions[j]);
					fwdKin_us = fkTimer.stop(); // Getting number of microseconds as a double.
				}
				++fkCalls;
				sumFwdKin_us = sumFwdKin_us + fwdKin_us;
				poseError = tipPoseInput - tipPoseOutput;
				for (unsigned int k = 0; k < 3; k++)
					sumAbsPos[k] += std::abs(poseError.m_pos[k]);
				sumOrientationError += geodesicOrientationError(tipPoseInput, tipPoseOutput);
			}
		}
		avgInvKin_us = sumInvKin_us / ITERATIONS;
		avgFwdKin_us = fkCalls > 0 ? sumFwdKin_us / fkCalls : std::chrono::duration<double, std::micro>::zero();
		double avgOrientationError = 0.0;
		if (fkCalls > 0)
		{
			for (unsigned int k = 0; k < 3; k++)
				avgPoseError.m_pos[k] = static_cast<float>(sumAbsPos[k] / fkCalls);
			avgOrientationError = sumOrientationError / fkCalls;
		}

		std::cout << "Generated " << ITERATIONS << " random valid tip poses for " << robot.getRobotType() << "..."
				  << '\n';
		std::cout << "Generated eight inverse kinematic solutions for each tip pose..." << '\n';
		std::cout << "Average IK function execution time: " << avgInvKin_us.count() << "us" << '\n';
		std::cout << "Each IK sol (" << fkCalls << ") has been run through forward kinematics..." << '\n';
		std::cout << "Average FK function execution time: " << avgFwdKin_us.count() << "us" << '\n';
		std::cout << "position error = mean |tip_pose_input - tip_pose_output| over all valid IK solutions" << '\n';
		std::cout << "average position error (m): " << avgPoseError.m_pos[0] << " " << avgPoseError.m_pos[1] << " "
				  << avgPoseError.m_pos[2] << '\n';
		// Rotation-invariant SO(3) geodesic distance, not a raw Euler-component
		// subtraction -- see #54 (quirk Q5: FK-extract / IK-compose Euler conventions
		// are asymmetric, so a per-component Euler diff is not a meaningful metric).
		std::cout << "average orientation error, SO(3) geodesic distance (rad): " << avgOrientationError << '\n';
	}

	std::chrono::microseconds timer::stop()
	{
		m_endTimePoint = std::chrono::high_resolution_clock::now();

		auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTimePoint);
		auto end = std::chrono::time_point_cast<std::chrono::microseconds>(m_endTimePoint);

		return end - start;
	}

} // namespace benchmark
