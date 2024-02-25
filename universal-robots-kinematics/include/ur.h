#pragma once

#include <map>
#include <string>
#include <vector>

#include "ur_parameters.h"
#include "my_math.h"

namespace universal_robots_kinematics
{
	class UR
	{
	private:
		RobotParameters parameters;
		std::vector<my_math::Joint> joints;
		Eigen::Matrix<float, RobotParameters::numReferenceFrames, 4> mdhMatrix; // Modified Denavit-Hartenberg matrix
		Eigen::Matrix4f individualTransformationMatrices[RobotParameters::numReferenceFrames]; // Array of(individual) transformation matrices iTi + 1 / i - 1Ti
		Eigen::Matrix4f generalTransformationMatrices[RobotParameters::numReferenceFrames] = {}; // Array of (general) transformation matrices 0Ti
	public:
		UR(const RobotParameters& params);
		~UR() = default;

		RobotParameters getParameters() const;
		void setParameters(const RobotParameters& params);

		my_math::Pose forwardKinematics(const std::vector<double>& targetJointAnglesInRad);
	private:
		void setTheta(const std::vector<double>& jointVal);
		void setModifiedDenavitHartenbergMatrix();
	};
}