// universalRobotsKinematics.cpp

#include "universalRobotsKinematics.h"



namespace universalRobots
{
	/////////////////////////////////////////////////////////////////////////////
	/// UR methods
	/// 

	
	/// UR constructor
	/// 
	/// Only constructor available for now.
	/// User is only allowed to specify whether there is an end-effector and its translation to the tip.
	/// Example usage:
	///		universalRobots::UR robot_one(); // Robot does not have an end-effector.
	///		universalRobots::UR robot_one(true, 0.15f); // End-effector translated 0.15 meters from the robot's tip.
	/// 
	UR::UR(const bool& endEffector, const float& endEffectorDimension)
		: m_endEffector(endEffector)
	{
		m_d[m_numTransZ - 1] = endEffectorDimension;
		m_MDHmatrix <<  0.0f,				0.0f,		m_d[0],			m_theta[0] ,					// 0T1
						mathLib::rad(-90),	0.0f,		m_d[1],			m_theta[1] + mathLib::rad(-90) ,// 1T2
						0.0f,				m_a[0],		m_d[2],			m_theta[2] ,					// 2T3
						0.0f,				m_a[1],		m_d[3],			m_theta[3] ,					// 3T4
						0.0f,				m_a[2],		m_d[4],			mathLib::rad(90) ,				// 4T4'
						mathLib::rad(90),	0.0f,		0.0f,			m_theta[4] ,					// 4'T5
						mathLib::rad(-90),	0.0f,		0.0f,			mathLib::rad(-90) ,				// 5T5'
						0.0f,				m_a[3],		m_d[5],			m_theta[5] ,					// 5'T6
						0.0f,				0,			m_d[6],			0 ;								// 6T7
	}

	/// setRobotType
	/// 
	void UR::setRobotType(const URtype& type)
	{
		m_type = type;
	}

	/// setTransZ
	/// 
	void UR::setTransZ(const float(&d)[])
	{
		memcpy(m_d, d, sizeof(m_d));
	}

	/// setTransX
	/// 
	void UR::setTransX(const float(&a)[])
	{
		memcpy(m_a, a, sizeof(m_a));
	}

	/// setMDHmatrix
	/// 
	void UR::setMDHmatrix(void)
	{
		m_MDHmatrix << 0.0f,					0.0f,		m_d[0],			m_theta[0] ,					// 0T1
						mathLib::rad(-90),		0.0f,		m_d[1],			m_theta[1] + mathLib::rad(-90) ,// 1T2
						0.0f,					m_a[0],		m_d[2],			m_theta[2] ,					// 2T3
						0.0f,					m_a[1],		m_d[3],			m_theta[3] ,					// 3T4
						0.0f,					m_a[2],		m_d[4],			mathLib::rad(90) ,				// 4T4'
						mathLib::rad(90),		0.0f,		0.0f,			m_theta[4] ,					// 4'T5
						mathLib::rad(-90),		0.0f,		0.0f,			mathLib::rad(-90) ,				// 5T5'
						0.0f,					m_a[3],		m_d[5],			m_theta[5] ,					// 5'T6
						0.0f,					0,			m_d[6],			0 ;								// 6T7
	}	
	
	/// setTheta
	/// 
	void UR::setTheta(const float(&jointVal)[])
	{
		std::memcpy(m_theta, jointVal, sizeof(m_theta));
	}

	/// setTipPose
	/// 
	void UR::setTipPose(const mathLib::tipPose& newTipPose)
	{
		m_tipPose = newTipPose;
	}

	/// getRobotType
	/// 
	/// Returns the enum URtype. Used for printing purposes.
	/// 
	const URtype UR::getRobotType(void) const
	{
		return m_type;
	}

	/// getTransZ
	/// 
	/// Returns the values of the z-axis translations (d array). Used for printing purposes.
	/// 
	const float* UR::getTransZ(void) const
	{
		return m_d;
	}


	/// getTransX
	/// 
	/// Returns the values of the x-axis translations (a array). Used for printing purposes.
	/// 
	const float* UR::getTransX(void) const
	{
		return m_a;
	}

	/// getTheta
	/// 
	/// Returns the robot's current joint values. Used for printing purposes.
	/// 
	const float* UR::getTheta(void) const
	{
		return m_theta;
	}

	/// getTipPose
	/// 
	/// Returns the robot's current tip pose. Used for printing purposes.
	/// 
	const mathLib::tipPose UR::getTipPose(void) const
	{
		return m_tipPose;
	}

	/// forwardKinematics
	/// 
	/// Receives an array of target joint values and computes the pose of the robot's tip.
	/// 
	mathLib::tipPose UR::forwardKinematics(const float(&targetJointVal)[])
	{
		// Assign joint values to compute MDH matrix.
		setTheta(targetJointVal);
		setMDHmatrix();

		// Create an array of (individual) transformation matrices iTi+1 / i-1Ti
		Eigen::Matrix4f individualTransformationMatrices[m_numReferenceFrames] = {};
		// Determine the indiviual transformation matrices.
		for (unsigned int i = 0; i < m_numReferenceFrames; i++)
		{
			individualTransformationMatrices[i] = mathLib::calcTransformationMatrix(m_MDHmatrix.row(i));
		}

		// Create another array of transformation matrices 0Ti
		Eigen::Matrix4f generalTransformationMatrices[m_numReferenceFrames] = {};
		// Determine the general transformation matrices.
		generalTransformationMatrices[0] = individualTransformationMatrices[0];
		for (unsigned int i = 1; i < m_numReferenceFrames; i++)
		{
			generalTransformationMatrices[i] = generalTransformationMatrices[i - 1] * individualTransformationMatrices[i];
		}

		Eigen::Matrix3f rotationMatrix;
		rotationMatrix << generalTransformationMatrices[m_numReferenceFrames - 1](0, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](0, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](0, 2),
						  generalTransformationMatrices[m_numReferenceFrames - 1](1, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](1, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](1, 2),
						  generalTransformationMatrices[m_numReferenceFrames - 1](2, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](2, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](2, 2);
		
		return mathLib::tipPose( Eigen::Matrix<float, 1, 3>(generalTransformationMatrices[m_numReferenceFrames - 1](0, 3), 
															generalTransformationMatrices[m_numReferenceFrames - 1](1, 3), 
															generalTransformationMatrices[m_numReferenceFrames - 1](2, 3)), 
								rotationMatrix);
	}
	

	float** UR::inverseKinematics(const Eigen::Matrix<float, 1, 6>& targetTipPose)
	{
		float ikSols[m_numIkSol][m_numDoF] = {};

		Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();

		// Get translation matrix
		const Eigen::Vector3f translation = { targetTipPose(0, 0), targetTipPose(0, 1), targetTipPose(0, 2) };
		
		// Calculate rotation matrix using Eigen 
		const Eigen::AngleAxisf rollAngle(targetTipPose(0, 3), Eigen::Vector3f::UnitZ());
		const Eigen::AngleAxisf yawAngle(targetTipPose(0, 4), Eigen::Vector3f::UnitY());
		const Eigen::AngleAxisf pitchAngle(targetTipPose(0, 5), Eigen::Vector3f::UnitX());

		const Eigen::Quaternion<float> auxQuaternion = rollAngle * pitchAngle * yawAngle;
		const Eigen::Matrix3f rotation = auxQuaternion.matrix();

		transformationMatrix.block<3, 3>(0, 0) = rotation;
		transformationMatrix.block<3, 1>(0, 3) = translation;

		
		// Computing theta1
		Eigen::Matrix<float, 1, 4> OP5 = transformationMatrix * Eigen::Matrix<float, 1, 4>(0, 0.0f, -m_d[5] - m_d[6], 1.0f).transpose(); // 0P5 position of reference frame {5} in relation to {0}
		const float theta1_psi = atan2(OP5(0, 1), OP5(0, 0));

		// There are two possible solutions for theta1, that depend on whether
		// the shoulder joint(joint 2) is left or right
		const float theta1_phi = acos( (m_d[1] + m_d[2] + m_d[3] + m_d[4]) / (sqrt( pow(OP5(0, 1), 2) + pow(OP5(0, 0), 2) ) ) );

		for (int i = -4; i < int (m_numIkSol) - 4 ; i++)
		{
			ikSols[i + 4][0] = (std::numbers::pi_v<float> / 2 + theta1_psi + (i < 0 ? 1 : -1)* theta1_phi) - std::numbers::pi_v<float>; // (i < 0 ? 1 : -1) first 4 theta1 values have positive phi
		}

			std::cout << theta1_phi;

		return reinterpret_cast<float**>(ikSols);
	}




	/////////////////////////////////////////////////////////////////////////////
	/// Operator overloading functions
	/// 

	/// operator <<
	/// 
	/// << overloading to be able to print a URtype enum.
	/// 
	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype& type)
	{
		switch (type)
		{
		case universalRobots::UR3:
			stream << "UR3";
			break;
		case universalRobots::UR5:
			stream << "UR5";
			break;
		case universalRobots::UR10:
			stream << "UR10";
			break;
		default:
			stream.setstate(std::ios_base::failbit);
		}
		return stream;
	}

	/// operator <<
	///
	/// << overloading to be able to print a UR object.
	/// 
	std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot)
	{
		stream << "Robot type: " << robot.getRobotType() << std::endl
			<< "Number of DoFs: " << robot.m_numDoF << std::endl
			<< "Link dimensions\n" << "Translations in the z-axis (meters)\n";
		for (unsigned int i = 0; i < robot.m_numTransZ; i++)
			stream << "d" << i + 1 << ": " << robot.getTransZ()[i] << std::endl;
		stream << "Translations in the x-axis (meters)\n";
		for (unsigned int i = 0; i < robot.m_numTransX; i++)
			stream << "a" << i + 2 << ": " << robot.getTransX()[i] << std::endl;
		//stream << "Modified Denavit-Hartengerg Matrix\n";
		//for (int x = 0; x < robot.m_numReferenceFrames; x++)  // loop lines
		//{
		//	for (int y = 0; y < 4; y++)  // loop columns
		//	{
		//		//stream << robot.getMDHmatrix()[x][y];
		//		stream << (robot.m_MDHmatrix[x][y]) << "				";
		//	}
		//	stream << endl;  
		//}
		stream << "Joint values (degress)\n";
		for (unsigned int i = 0; i < robot.m_numDoF; i++)
			stream << "Theta" << i + 1 << ": " << mathLib::deg(robot.getTheta()[i]) << std::endl;
		stream << "Tip pose:\n";
		stream << "x " << robot.getTipPose().m_pos.x() << " y " << robot.getTipPose().m_pos.y() << " z " << robot.getTipPose().m_pos.z() << " (meters)\nalpha "
			<< mathLib::deg(robot.getTipPose().m_rpy.x()) << " beta " << mathLib::deg(robot.getTipPose().m_rpy.y()) << " gamma " << mathLib::deg(robot.getTipPose().m_rpy.z()) << " (degrees)" << std::endl;
		return stream;
	}

} // namespace universalRobots 




