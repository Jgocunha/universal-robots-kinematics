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
	
	//float** UR::inverseKinematics(const Eigen::Matrix<float, 1, 6>& targetTipPose)
	float** UR::inverseKinematics(const Eigen::Matrix<float, 1, 6>& targetTipPose)
	{
		float ikSols[m_numIkSol][m_numDoF] = {};

		Eigen::Matrix4f T_07 = Eigen::Matrix4f::Identity(); // 0T7

		// Get translation matrix.
		const Eigen::Vector3f translation = { targetTipPose(0, 0), targetTipPose(0, 1), targetTipPose(0, 2) };

		// Calculate rotation matrix.
		Eigen::Matrix3f rotation;
		rotation = Eigen::AngleAxisf(targetTipPose(0, 3), Eigen::Vector3f::UnitX())
				*Eigen::AngleAxisf( targetTipPose(0, 4), Eigen::Vector3f::UnitY())
				*Eigen::AngleAxisf( targetTipPose(0, 5), Eigen::Vector3f::UnitZ());

		T_07.block<3, 3>(0, 0) = rotation;
		T_07.block<3, 1>(0, 3) = translation;

		// Computing theta1.
		Eigen::Matrix<float, 1, 4> P_05 = T_07 * Eigen::Matrix<float, 1, 4>(0.0f, 0.0f, -m_d[5] - m_d[6], 1.0f).transpose(); // 0P5 position of reference frame {5} in relation to {0}
		const float theta1_psi = atan2(P_05(0, 1), P_05(0, 0));

		// There are two possible solutions for theta1, that depend on whether the shoulder joint (joint 2) is left or right.
		const float theta1_phi = acos( (m_d[1] + m_d[2] + m_d[3] + m_d[4]) / (sqrt( pow(P_05(0, 1), 2) + pow(P_05(0, 0), 2) ) ) );

		for (int i = -4; i < int (m_numIkSol) - 4 ; i++)
		{
			ikSols[i + 4][0] = (std::numbers::pi_v<float> / 2 + theta1_psi + (i < 0 ? 1 : -1)* theta1_phi) - std::numbers::pi_v<float>; // (i < 0 ? 1 : -1) first 4 theta1 values have positive phi
		}
		
		Eigen::Matrix4f T_06 = T_07;
		T_06(2, 3) = T_07(2, 3) - m_d[m_numTransZ - 1]; // 0T6

		for (unsigned int i = 0; i < m_numIkSol; i++)
		{
			// Computing theta5.
			 Eigen::Matrix4f T_01 = mathLib::calcTransformationMatrix(Eigen::RowVector4f{ 0.0f, 0.0f, m_d[0], ikSols[i][0] }); // Knowing theta1 it is possible to know 0T1
			 Eigen::Matrix4f T_16 = T_01.inverse() * T_06; // 1T6 = 1T0 * 0T6
			// There are two possible solutions for theta5, that depend on whether the wrist joint is up or down.
			if (i == 0 || i == 1 || i == 4 || i == 5) //(0, 1, 4, 5)
				ikSols[i][4] =  acos( (T_16(1, 3) - (m_d[1] + m_d[2] + m_d[3] + m_d[4])) / m_d[5] );
			else
				ikSols[i][4] = - acos( (T_16(1, 3) - (m_d[1] + m_d[2] + m_d[3] + m_d[4])) / m_d[5] );

			// Computing theta6.
			if (ikSols[i][4] == 0 || ikSols[i][4] == 2 * std::numbers::pi_v<float>) // If theta5 is equal to zero.
				ikSols[i][5] = 0.0f; // Give arbitrary value to theta6
			else
				ikSols[i][5] = std::numbers::pi_v<float> / 2 + atan2(-T_16.inverse()(1, 1) / sin(ikSols[i][4]), T_16.inverse()(0, 1) / sin(ikSols[i][4]));

			// Computing theta3, theta2, and theta4.

			// T_45 = T_44'*T_4'5
			 Eigen::Matrix4f T_44_ = mathLib::calcTransformationMatrix(Eigen::RowVector4f (0.0f, m_a[2], m_d[4], std::numbers::pi_v<float> / 2));
			 Eigen::Matrix4f T_4_5 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(std::numbers::pi_v<float> / 2, 0.0f, 0.0f, ikSols[i][4]));
			 Eigen::Matrix4f T_45 = T_44_ * T_4_5;

			// T_56 = T_55'*T_5'6
			 Eigen::Matrix4f T_55_ = mathLib::calcTransformationMatrix(Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, 0.0f, -std::numbers::pi_v<float> / 2));
			 Eigen::Matrix4f T_5_6 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(0.0f, m_a[3], m_d[5], ikSols[i][5]));
			 Eigen::Matrix4f T_56 = T_55_ * T_5_6;

			 Eigen::Matrix4f T_46 = T_45 * T_56;
			 Eigen::Matrix4f T_14 = T_16 * T_46.inverse();

			 float P_14_xz = sqrtf( pow(T_14(0, 3), 2) + pow(T_14(2, 3), 2));
			 float theta3_psi = acos((pow(P_14_xz, 2) - pow(m_a[1], 2) - pow(m_a[0], 2)) / (-2 * m_a[0] * m_a[1]));

			// Elbow up or down
			if ((i + 1) % 2 == 0)
			{
				// Computing theta3.
				//float theta3_psi = acos((pow(P_14_xz, 2) - pow(m_a[1], 2) - pow(m_a[0], 2)) / (-2 * m_a[0] * m_a[1]));
				ikSols[i][2] = std::numbers::pi_v<float> - theta3_psi;
				// Computing theta2.
				ikSols[i][1] = std::numbers::pi_v<float> / 2 - atan2(T_14(2, 3), T_14(0, 3)) + asin((m_a[1] * sin(-theta3_psi)) / P_14_xz);
				//joint(j, 2) = pi / 2 - atan2(P_14(3), P_14(1)) + asin((a(3) * sin(psi)) / P_14_xz);
				// Computing theta4.
				 Eigen::Matrix4f T_12 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, m_d[1], ikSols[i][1] - std::numbers::pi_v<float> / 2));
				 Eigen::Matrix4f T_23 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(0.0f, m_a[0], m_d[2], ikSols[i][2]));
				 Eigen::Matrix4f T_03 = T_01 * T_12 * T_23;

				 Eigen::Matrix4f T_36 = T_03.inverse() * T_06;
				 Eigen::Matrix4f T_34 = T_36 * T_46.inverse();

				ikSols[i][3] = atan2(T_34(1, 0), T_34(0, 0));
			}
			else
			{
				// Computing theta3.
				//float theta3_psi = acos((pow(P_14_xz, 2) - pow(m_a[1], 2) - pow(m_a[0], 2)) / (-2 * m_a[0] * m_a[1]));
				ikSols[i][2] = std::numbers::pi_v<float> + theta3_psi;
				// Computing theta2.
				ikSols[i][1] = std::numbers::pi_v<float> / 2 - atan2(T_14(2, 3), T_14(0, 3)) + asin(m_a[1] * sin(theta3_psi) / P_14_xz);
				// joint(j,2) = pi/2 - (atan2( P_14(3), +P_14(1)) + asin( (a(3)*sin(-psi))/P_14_xz));
				// Computing theta4.
				 Eigen::Matrix4f T_12 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, m_d[1], ikSols[i][1] - std::numbers::pi_v<float> / 2));
				 Eigen::Matrix4f T_23 = mathLib::calcTransformationMatrix(Eigen::RowVector4f(0.0f, m_a[0], m_d[2], ikSols[i][2]));
				 Eigen::Matrix4f T_03 = T_01 * T_12 * T_23;

				 Eigen::Matrix4f T_36 = T_03.inverse() * T_06;
				 Eigen::Matrix4f T_34 = T_36 * T_46.inverse();

				ikSols[i][3] = atan2(T_34(1, 0), T_34(0, 0));
			}
		}

		for (unsigned int i = 0; i < m_numIkSol; i++)
			std::cout << "IK solution " << i << ": " << mathLib::deg(ikSols[i][0]) << " " << mathLib::deg(ikSols[i][1]) << " " << mathLib::deg(ikSols[i][2]) << " " <<
			mathLib::deg(ikSols[i][3]) << " " << mathLib::deg(ikSols[i][4]) << " " << mathLib::deg(ikSols[i][5]) << std::endl;

		return reinterpret_cast<float **>(ikSols);
		//return;
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




