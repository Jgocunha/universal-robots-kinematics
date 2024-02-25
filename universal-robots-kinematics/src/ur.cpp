#include "ur.h"

namespace universal_robots_kinematics
{

	UR::UR(const RobotParameters& params)
	: parameters(params)
	{
		for (int i = 0; i < RobotParameters::numDoF; i++)
			joints.push_back(my_math::Joint(my_math::Pose{}, 0));
		setModifiedDenavitHartenbergMatrix();
		//individualTransformationMatrices;
	}

	RobotParameters UR::getParameters() const
	{
		return parameters;
	}

	void UR::setParameters(const RobotParameters& params)
	{
		parameters = params;
	}

	my_math::Pose UR::forwardKinematics(const std::vector<double>& targetJointAnglesInRad)
	{
		// Assign joint values to compute MDH matrix.
		setTheta(targetJointAnglesInRad);
		setModifiedDenavitHartenbergMatrix();

		// Determine the individual transformation matrices.
		for (unsigned int i = 0; i < RobotParameters::numReferenceFrames; i++)
			individualTransformationMatrices[i] = my_math::calcTransformationMatrix(mdhMatrix.row(i));

		// Determine the general transformation matrices.		
		Eigen::Matrix3f rotationMatrix;
		unsigned int currentJoint = 0;
		my_math::Pose tipPose = {};
		for (unsigned int i = 0; i < RobotParameters::numReferenceFrames; i++)
		{
			if (!i)
				generalTransformationMatrices[0] = individualTransformationMatrices[0];
			else
				generalTransformationMatrices[i] = generalTransformationMatrices[i - 1] * individualTransformationMatrices[i];


			// Obtaining the joint pose
			// Since we have more reference frames than joints only some represent a joint pose
			// 0T1 J1 // 1T2 J2// 2T3 J3// 3T4 J4// 4T4' XX// 4'T5 J5// 5T5' XX// 5'T6 J6// 6T7 tipPose
			if (i == 0 || i == 1 || i == 2 || i == 3 || i == 5 || i == 7 || i == 8)
			{
				// 0T1 1 // 1T2 2// 2T3 3// 3T4 4// 4T4' // 4'T5 5// 5T5' // 5'T6 6// 6T7
				rotationMatrix << generalTransformationMatrices[i](0, 0), generalTransformationMatrices[i](0, 1), generalTransformationMatrices[i](0, 2),
					generalTransformationMatrices[i](1, 0), generalTransformationMatrices[i](1, 1), generalTransformationMatrices[i](1, 2),
					generalTransformationMatrices[i](2, 0), generalTransformationMatrices[i](2, 1), generalTransformationMatrices[i](2, 2);

				const Eigen::Vector3f position = { generalTransformationMatrices[i](0, 3), generalTransformationMatrices[i](1, 3), generalTransformationMatrices[i](2, 3) };

				// Tip pose
				if (i == 8)
					tipPose = my_math::Pose(position, rotationMatrix);
				else
				{
					joints[currentJoint] = my_math::Pose(position, rotationMatrix);
					currentJoint++;
				}
			}
		}

		return tipPose;
	}

	void UR::setTheta(const std::vector<double>& jointVal)
	{
		if (jointVal.size() != RobotParameters::numDoF) 
			throw std::invalid_argument("jointVal must have exactly " + std::to_string(RobotParameters::numDoF) + " elements.");

		for (int i = 0; i < RobotParameters::numDoF; i++)
				joints[i].value = jointVal[i];
	}

	void UR::setModifiedDenavitHartenbergMatrix()
	{
		mdhMatrix <<
		
			0,					0,			parameters.linkDimensions.d[0],			joints[0].value,
			my_math::degToRad(-90.0),	0,			parameters.linkDimensions.d[1],			joints[1].value + my_math::degToRad(-90.0),
			0,					parameters.linkDimensions.a[0],		parameters.linkDimensions.d[2],			joints[2].value,
			0,					parameters.linkDimensions.a[1],		parameters.linkDimensions.d[3],			joints[3].value,
			0,					parameters.linkDimensions.a[2],		parameters.linkDimensions.d[4],			my_math::degToRad(90.0),
			my_math::degToRad(90.0),	0,			0,				joints[4].value,
			my_math::degToRad(-90.0),	0,			0,				my_math::degToRad(-90.0),
			0,					parameters.linkDimensions.a[3],		parameters.linkDimensions.d[5],			joints[5].value,
			0,					0,			parameters.linkDimensions.d[6],			0;
	}
}


